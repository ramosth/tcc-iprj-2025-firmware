/*
 * =====================================================================================
 *  SISTEMA DE ALERTA PARA BARRAGEM DE REJEITOS — PROTÓTIPO FÍSICO (ESP8266) — versão 21
 *  TCC UERJ-IPRJ | Thamires Ramos dos Santos
 *  Base: tcc_versao_20.ino (guardado como backup) + lógica do sketch_v3 (Wokwi).
 *
 *  Modelo de alerta (limiar hidrometeorológico de duas variáveis) — igual à simulação:
 *    Eq. 1  S   = (L - L_seco) / (L_sat - L_seco)            saturação relativa (0 a 1)
 *    Eq. 2  P72 = P48,obs + P24,prev                          chuva da janela de 72 h
 *    Eq. 3  VERMELHO se S >= S2 e P72 >= P2
 *           AMARELO  se S >= S1 e P72 >= P1 (e não vermelho)
 *           VERDE    nos demais casos
 *  Contingências:
 *    higrômetro com falha -> só chuva: P72 >= P1 AMARELO; P72 >= P2 VERMELHO
 *    chuva incompleta     -> parcela não obtida OU dia sem registro (null) no BNDMET:
 *                            mínimo garantido = soma do obtido; se >= P1, regra com o mínimo;
 *                            se < P1, como "sem dados de chuva"
 *    sem dados de chuva   -> só solo:  S >= S2 AMARELO; senão VERDE ("sem dados de chuva")
 *    sem os dois          -> AMARELO por precaução (decisão do orientador)
 *
 *  Diferenças em relação à simulação (sketch_v3, ESP32):
 *    - Higrômetro real (FC-28 + LM393) no pino A0, ADC de 10 bits (0 a 1023).
 *    - Sem supervisão de energia: alimentação externa de 5 V pela porta USB-C.
 *      O JSON não leva os campos rede, vbat e estadoBateria (o backend grava null).
 *    - Modo demonstração (FATOR_DEMO): divide os intervalos de 60 min / 10 min, como no
 *      sketch_v3. Com FATOR_DEMO = 1, o firmware opera nos tempos reais.
 *      O comando "chuva <mm>" continua rodando um ciclo na hora.
 *
 *  Hardware (montagem do Fritzing, versão 3):
 *    Higrômetro A0 | LED azul D1 (nível VERDE) | LED amarelo D2 | LED vermelho D3
 *    Buzzer ativo TMB-12A03 D4 (nível alto = som)
 *
 *  Comandos serial: status | calibrar [seco|sat|reset] | chuva <mm>|parcial <mm>|off|na | help
 *  Chaves e endereços: secrets.h (não publicar). Modelo: secrets.h.example.
 *  Placa na Arduino IDE: "NodeMCU 1.0 (ESP-12E Module)", porta COM3, 115200 baud.
 * =====================================================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <time.h>
#include "secrets.h" // WIFI_SSID, WIFI_PASS, API_BASE_URL, BNDMET_API_KEY, OWM_API_KEY

// =====================================================================================
//  PINOS (montagem física — os mesmos do tcc_versao_20)
// =====================================================================================
const uint8_t PIN_HIGROMETRO = A0;   // Fonte: espressif_esp8266ex (Seção 4.9, único ADC, pino TOUT = A0)
// Números GPIO (e não D1..D4): os apelidos D1..D4 só existem quando a placa escolhida na
// IDE é "NodeMCU 1.0 (ESP-12E Module)". Com GPIO o código compila em qualquer placa ESP8266.
// Mapa da NodeMCU: D1 = GPIO5 | D2 = GPIO4 | D3 = GPIO0 | D4 = GPIO2 (os fios continuam nos mesmos pinos).
const uint8_t PIN_LED_VERDE = 5;     // D1 — Fonte: montagem (Fritzing v3) — LED AZUL indica o nível VERDE
const uint8_t PIN_LED_AMARELO = 4;   // D2 — Fonte: montagem (Fritzing v3)
const uint8_t PIN_LED_VERMELHO = 0;  // D3 — Fonte: montagem (Fritzing v3)
const uint8_t PIN_BUZZER = 2;        // D4 — Fonte: quickteck_tmb12a03 (buzzer ativo: nível alto = som)

// =====================================================================================
//  CONSTANTES DO MODELO (Eq. 1–3) — idênticas ao sketch_v3
// =====================================================================================
const int ADC_MAX = 1023;      // Fonte: espressif_esp8266ex (ADC SAR de 10 bits: 0 a 1023)
const float S1 = 0.64f;        // Fonte: mirus2018developing (Tabela 1, Seattle: limiar inferior de saturação)
const float S2 = 0.86f;        // Fonte: mirus2018developing (Tabela 1, Seattle: limiar superior de saturação)
const float P1_MM = 60.0f;     // Fonte: mendes2020proposicao (72 h, nível de atenção, Campos do Jordão)
const float P2_MM = 100.0f;    // Fonte: mendes2020proposicao (72 h, nível de alerta, Campos do Jordão)
const int DIAS_P48 = 2;        // Fonte: mirus2018developing (janela de t-48 h; 2 totais diários I006)
const int BLOCOS_P24 = 8;      // Fonte: mirus2018developing (janela até t+24 h) + openweathermap_forecast5 (8 x 3 h)

// Calibração padrão (substituída pelo comando "calibrar" e gravada na EEPROM).
// No higrômetro resistivo, solo seco = leitura alta; a Eq. 1 trata a escala invertida.
const int L_SECO_PADRAO = 1023; // Fonte: calibração da autora (tcc_versao_20, SENSOR_SECO)
const int L_SAT_PADRAO = 240;   // Fonte: calibração da autora (tcc_versao_20, SENSOR_UMIDO)
const int EEPROM_TAM = 512;     // Fonte: software (área de EEPROM emulada)
const int END_SECO = 0;         // Fonte: software (mesmo endereço do tcc_versao_20)
const int END_SAT = 4;          // Fonte: software (mesmo endereço do tcc_versao_20)

// =====================================================================================
//  INTERVALOS DE LEITURA E ENVIO
// =====================================================================================
// MODO DEMONSTRAÇÃO — divide os intervalos de leitura por FATOR_DEMO (igual ao sketch_v3).
//   FATOR_DEMO = 1   -> tempos reais (padrão publicado): 60 min (sem chuva) / 10 min (com chuva ou alerta)
//   FATOR_DEMO = 120 -> 30 s / 5 s   (testes e prints)
//   FATOR_DEMO = 12  -> 5 min / 50 s
// É recurso de TESTE; não é parâmetro do modelo. Volte para 1 na operação.
const uint32_t FATOR_DEMO = 1; // Fonte: decisão de projeto (modo de teste; nos testes do TCC foi usado 120)

const uint32_t INTERVALO_NORMAL_MS = 60UL * 60UL * 1000UL / FATOR_DEMO; // Fonte: freitas2021analise (60 min sem chuva)
const uint32_t INTERVALO_EVENTO_MS = 10UL * 60UL * 1000UL / FATOR_DEMO; // Fonte: freitas2021analise (10 min com chuva)

// =====================================================================================
//  PARÂMETROS DE SOFTWARE (não são parâmetros do modelo; ficam só no código)
// =====================================================================================
const uint32_t BAUD = 115200;           // Fonte: software (mesma taxa do tcc_versao_20)
const uint32_t TIMEOUT_HTTP_MS = 15000; // Fonte: software (tempo máximo de espera por resposta HTTP)
const uint32_t TIMEOUT_WIFI_MS = 15000; // Fonte: software (tempo máximo para conectar ao Wi-Fi)
const long FUSO_S = -3L * 3600L;        // Fonte: software (horário de Brasília, UTC-3, para as datas do BNDMET)
const long DIA_S = 86400L;              // Fonte: software (segundos em um dia)
// Aviso de espera entre ciclos: a cada 1 min nos tempos reais; desligado se o ciclo é curto.
const uint32_t AVISO_ESPERA_MS = 60000UL; // Fonte: software (mostra a cada 1 min quanto falta para o próximo ciclo)

const char *BNDMET_URL = "https://api-bndmet.decea.mil.br/v1/estacoes/D6594/fenomenos/I006"; // Fonte: decea_bndmet (estação D6594, I006)
const char *OWM_URL = "https://pro.openweathermap.org/data/2.5/forecast?lat=-20.1433&lon=-44.1997&units=metric&cnt="; // Fonte: openweathermap_forecast5 (Brumadinho-MG)
const char *API_LEITURAS = "/leituras"; // Fonte: rota do backend (POST /leituras)

// =====================================================================================
//  TIPOS E ESTADO
// =====================================================================================
enum Nivel { VERDE, AMARELO, VERMELHO };

struct Leitura {
  int adc = 0;
  float S = 0;
  bool sensorOk = false;
  float p48 = 0, p24 = 0, p72 = 0;
  bool p48Ok = false, p24Ok = false;
  int p48DiasNulos = 0; // dias sem registro (null ou ausentes): P48 incompleto
  bool p72Injetado = false;
  Nivel nivel = VERDE;
};

Leitura leitura;
int lSeco = L_SECO_PADRAO;
int lSat = L_SAT_PADRAO;

// Modo de teste do comando "chuva": 0 = APIs reais; 1 = P72 injetado; 2 = sem dados de chuva;
// 3 = dados incompletos injetados ("chuva parcial <mm>": P48 = mm com 1 dia sem registro, P24 = 0)
int modoChuva = 0;
float p72Teste = 0;

uint32_t ultimoCicloMs = 0;
bool cicloForcado = true; // primeiro ciclo logo após o setup
String linhaSerial = "";

const char *textoNivel(Nivel n) {
  return n == VERMELHO ? "VERMELHO" : (n == AMARELO ? "AMARELO" : "VERDE");
}

// =====================================================================================
//  CALIBRAÇÃO (L_seco e L_sat na EEPROM)
// =====================================================================================
bool calibracaoValida(int seco, int sat) {
  return seco >= 0 && seco <= ADC_MAX && sat >= 0 && sat <= ADC_MAX && seco != sat;
}

void carregarCalibracao() {
  EEPROM.begin(EEPROM_TAM);
  int seco, sat;
  EEPROM.get(END_SECO, seco);
  EEPROM.get(END_SAT, sat);
  if (calibracaoValida(seco, sat)) { lSeco = seco; lSat = sat; }
  Serial.printf("[CALIB] L_seco=%d | L_sat=%d\n", lSeco, lSat);
}

void salvarCalibracao() {
  EEPROM.put(END_SECO, lSeco);
  EEPROM.put(END_SAT, lSat);
  EEPROM.commit();
}

// =====================================================================================
//  EQUAÇÃO 1 — SATURAÇÃO RELATIVA DO SOLO
// =====================================================================================
// Leitura no extremo da escala (0 ou 1023) = circuito aberto ou em curto -> falha.
// O núcleo Arduino do ESP8266 pode devolver 1024 no fim da escala; limita a 1023 (10 bits).
int lerAdc() {
  return constrain(analogRead(PIN_HIGROMETRO), 0, ADC_MAX);
}

bool leituraValida(int adc) {
  return adc > 0 && adc < ADC_MAX;
}

float calcularSaturacao(int adc) {
  if (lSat == lSeco) return 0.0f;
  float s = (float)(adc - lSeco) / (float)(lSat - lSeco);
  return constrain(s, 0.0f, 1.0f);
}

void lerSaturacao() {
  leitura.adc = lerAdc();
  leitura.sensorOk = leituraValida(leitura.adc) && (lSat != lSeco);
  leitura.S = leitura.sensorOk ? calcularSaturacao(leitura.adc) : 0.0f;
}

// =====================================================================================
//  CONECTIVIDADE
// =====================================================================================
bool garantirWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.print("[WiFi] Conectando");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < TIMEOUT_WIFI_MS) {
    delay(250); // Fonte: software (espera entre tentativas)
    Serial.print(".");
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  Serial.println(ok ? " OK" : " FALHOU");
  if (ok) Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
  return ok;
}

bool relogioSincronizado() {
  return time(nullptr) > 1600000000L; // Fonte: software (data posterior a 2020 = NTP sincronizado)
}

void sincronizarRelogio() {
  configTime(FUSO_S, 0, "pool.ntp.org", "time.nist.gov");
  uint32_t t0 = millis();
  while (!relogioSincronizado() && millis() - t0 < TIMEOUT_WIFI_MS) delay(250); // Fonte: software (espera NTP)
  Serial.println(relogioSincronizado() ? "[NTP] Relogio sincronizado" : "[NTP] Sem sincronizacao");
}

String formatarData(time_t t) {
  struct tm *tm = localtime(&t);
  char buf[11];
  strftime(buf, sizeof(buf), "%Y-%m-%d", tm);
  return String(buf);
}

// GET HTTPS; devolve o corpo ou "" em caso de erro.
String httpsGet(const String &url, const char *chaveBndmet) {
  BearSSL::WiFiClientSecure cliente;
  cliente.setInsecure(); // sem verificação de certificado (mesma escolha do tcc_versao_20)
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);
  if (!http.begin(cliente, url)) return "";
  if (chaveBndmet) http.addHeader("x-api-key", chaveBndmet);
  int codigo = http.GET();
  String corpo = (codigo == HTTP_CODE_OK) ? http.getString() : "";
  if (codigo != HTTP_CODE_OK) Serial.printf("[HTTP] Falha %d em %s\n", codigo, url.substring(0, 60).c_str());
  http.end();
  return corpo;
}

// =====================================================================================
//  EQUAÇÃO 2 — P48 (BNDMET) + P24 (OpenWeatherMap)
// =====================================================================================
String textoDia(const String &data, bool existe, bool nulo, float mm) {
  String t = data + ": ";
  if (!existe) return t + "sem registro (ausente)";
  if (nulo) return t + "null (ausente)";
  return t + String(mm, 1) + " mm";
}

// P48: soma dos totais diários (I006) dos 2 últimos dias completos (anteontem e ontem).
// Com HTTP 200, a API respondeu. Dia com null (ou ausente da lista) é dado AUSENTE: fica
// fora da soma e é contado em diasNulos; P48 passa a ser um mínimo (incompleto).
// P48 só fica indisponível quando a consulta falha (HTTP/conexão/JSON).
bool buscarP48(float &p48, int &diasNulos) {
  if (!relogioSincronizado()) {
    Serial.println("[BNDMET I006] PULADO - horario ainda nao sincronizado.");
    return false;
  }
  time_t agora = time(nullptr);
  String dAnteontem = formatarData(agora - DIAS_P48 * DIA_S);
  String dOntem = formatarData(agora - DIA_S);
  String url = String(BNDMET_URL) + "?dataInicio=" + dAnteontem + "&dataFinal=" + dOntem;
  String corpo = httpsGet(url, BNDMET_API_KEY);
  if (corpo.isEmpty()) return false;

  JsonDocument doc;
  if (deserializeJson(doc, corpo)) { Serial.println("[BNDMET I006] FALHA - JSON invalido"); return false; }
  JsonArray dados = doc["data"]["data"].as<JsonArray>(); // pares [timestamp_ms, valor_ou_null]
  if (dados.isNull()) { Serial.println("[BNDMET I006] FALHA - campo data.data ausente"); return false; }

  bool existeA = false, nuloA = false, existeO = false, nuloO = false;
  float mmA = 0, mmO = 0;
  for (JsonArray par : dados) {
    if (par.size() < 2) continue;
    String dia = formatarData((time_t)(par[0].as<long long>() / 1000LL));
    bool nulo = par[1].isNull();
    float mm = nulo ? 0.0f : par[1].as<float>(); // null: não entra na soma (ausente)
    if (dia == dAnteontem) { existeA = true; nuloA = nulo; mmA = mm; }
    if (dia == dOntem) { existeO = true; nuloO = nulo; mmO = mm; }
  }
  diasNulos = (!existeA || nuloA) + (!existeO || nuloO);
  p48 = (existeA && !nuloA ? mmA : 0.0f) + (existeO && !nuloO ? mmO : 0.0f); // só dias com valor

  Serial.println("[BNDMET I006] anteontem " + textoDia(dAnteontem, existeA, nuloA, mmA) +
                 " | ontem " + textoDia(dOntem, existeO, nuloO, mmO));
  if (diasNulos == 0) Serial.println("[BNDMET I006] OK - P48 = " + String(p48, 1) + " mm");
  else Serial.println("[BNDMET I006] INCOMPLETO - P48 >= " + String(p48, 1) + " mm (" + String(diasNulos) +
                      " dia(s) sem registro na estacao)");
  return true;
}

// P24: soma de rain.3h (mm) nos 8 próximos blocos de 3 h. Bloco sem "rain" = 0 mm.
// O filtro guarda só o campo usado, para caber na memória do ESP8266.
bool buscarP24(float &p24) {
  String corpo = httpsGet(String(OWM_URL) + BLOCOS_P24 + "&appid=" + OWM_API_KEY, nullptr);
  if (corpo.isEmpty()) return false;

  JsonDocument filtro;
  filtro["list"][0]["rain"]["3h"] = true;
  JsonDocument doc;
  if (deserializeJson(doc, corpo, DeserializationOption::Filter(filtro))) return false;
  JsonArray lista = doc["list"].as<JsonArray>();
  if (lista.isNull() || (int)lista.size() < BLOCOS_P24) return false;

  float soma = 0;
  for (JsonObject bloco : lista) soma += bloco["rain"]["3h"] | 0.0f;
  p24 = soma;
  return true;
}

void obterChuva() {
  leitura.p72Injetado = false;
  leitura.p48DiasNulos = 0;
  if (modoChuva == 1) { // teste: P72 injetado pelo comando "chuva <mm>"
    leitura.p48Ok = leitura.p24Ok = true;
    leitura.p48 = leitura.p24 = 0;
    leitura.p72 = p72Teste;
    leitura.p72Injetado = true;
    return;
  }
  if (modoChuva == 2) { // teste: simula APIs fora do ar
    leitura.p48Ok = leitura.p24Ok = false;
    return;
  }
  if (modoChuva == 3) { // teste: dados incompletos (dia null no BNDMET); mínimo garantido = mm informado
    leitura.p48Ok = leitura.p24Ok = true;
    leitura.p48 = p72Teste;
    leitura.p24 = 0;
    leitura.p48DiasNulos = 1;
    leitura.p72 = p72Teste;
    leitura.p72Injetado = true;
    return;
  }
  bool rede = garantirWiFi();
  if (rede && !relogioSincronizado()) sincronizarRelogio();
  leitura.p48Ok = rede && buscarP48(leitura.p48, leitura.p48DiasNulos);
  leitura.p24Ok = rede && buscarP24(leitura.p24);
  if (leitura.p48Ok && leitura.p24Ok) leitura.p72 = leitura.p48 + leitura.p24; // Eq. 2
}

// Chuva completa: as duas parcelas obtidas e nenhum dia sem registro no BNDMET.
bool chuvaCompleta() {
  return leitura.p48Ok && leitura.p24Ok && leitura.p48DiasNulos == 0;
}

// Chuva incompleta: algo foi obtido, mas falta uma parcela ou um dia do BNDMET.
bool chuvaIncompleta() {
  return (leitura.p48Ok || leitura.p24Ok) && !chuvaCompleta();
}

// P72 mínimo garantido: soma do que foi obtido (a chuva ausente não é negativa).
float p72Minimo() {
  return (leitura.p48Ok ? leitura.p48 : 0.0f) + (leitura.p24Ok ? leitura.p24 : 0.0f);
}

// Chuva usada na regra: P72 completo, ou o mínimo garantido se ele já atinge P1.
// Devolve false quando a regra deve tratar como "sem dados de chuva".
bool chuvaParaRegra(float &p) {
  if (chuvaCompleta()) { p = leitura.p72; return true; }
  if (chuvaIncompleta() && p72Minimo() >= P1_MM) { p = p72Minimo(); return true; }
  return false;
}

// =====================================================================================
//  EQUAÇÃO 3 + CONTINGÊNCIAS — NÍVEL DE ALERTA (idêntico ao sketch_v3)
// =====================================================================================
Nivel nivelPorChuva(float p72) {
  if (p72 >= P2_MM) return VERMELHO;
  if (p72 >= P1_MM) return AMARELO;
  return VERDE;
}

Nivel avaliarNivel(const Leitura &l) {
  float p = 0;
  bool chuva = chuvaParaRegra(p);
  if (l.sensorOk && chuva) { // Eq. 3: limiar bilinear (S e P72 ao mesmo tempo)
    if (l.S >= S2 && p >= P2_MM) return VERMELHO;
    if (l.S >= S1 && p >= P1_MM) return AMARELO;
    return VERDE;
  }
  if (!l.sensorOk && chuva) return nivelPorChuva(p);              // higrômetro com falha: só chuva
  if (l.sensorOk && !chuva) return (l.S >= S2) ? AMARELO : VERDE; // caso especial
  return AMARELO; // sem higrômetro E sem chuva: AMARELO por precaução (decisão do orientador)
}

// =====================================================================================
//  ALERTA LOCAL (LEDs de nível e buzzer)
// =====================================================================================
void acionarAlertaLocal(Nivel n) {
  digitalWrite(PIN_LED_VERDE, n == VERDE ? HIGH : LOW);
  digitalWrite(PIN_LED_AMARELO, n == AMARELO ? HIGH : LOW);
  digitalWrite(PIN_LED_VERMELHO, n == VERMELHO ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, n == VERMELHO ? HIGH : LOW); // buzzer só no VERMELHO
}

// =====================================================================================
//  ENVIO JSON AO BACKEND (mesmo formato da simulação, sem os campos de energia)
// =====================================================================================
String montarJson() {
  JsonDocument doc;
  doc["evento"] = "ciclo";
  doc["nivel"] = textoNivel(leitura.nivel);
  if (leitura.sensorOk) doc["S"] = serialized(String(leitura.S, 2)); else doc["S"] = nullptr;
  if (leitura.p48Ok && !leitura.p72Injetado) doc["P48"] = leitura.p48; else doc["P48"] = nullptr;
  if (leitura.p24Ok && !leitura.p72Injetado) doc["P24"] = leitura.p24; else doc["P24"] = nullptr;
  if (chuvaCompleta()) doc["P72"] = leitura.p72; else doc["P72"] = nullptr;
  if (chuvaIncompleta()) doc["p72Minimo"] = p72Minimo(); // limite inferior de P72
  doc["p48DiasNulos"] = leitura.p48DiasNulos; // dias sem registro no BNDMET (P48 incompleto)
  doc["sensorIndisponivel"] = !leitura.sensorOk;
  doc["p48Indisponivel"] = !leitura.p48Ok;
  doc["p24Indisponivel"] = !leitura.p24Ok;
  doc["chuvaIncompleta"] = chuvaIncompleta();
  doc["semDadosChuva"] = !leitura.p48Ok && !leitura.p24Ok;
  doc["p72Injetado"] = leitura.p72Injetado; // true = cenário de teste (comando "chuva")
  doc["simulacao"] = false;                 // protótipo físico
  String saida;
  serializeJson(doc, saida);
  return saida;
}

void enviarLeitura() {
  String json = montarJson();
  Serial.println("[JSON] " + json);
  if (!garantirWiFi()) return;
  WiFiClient cliente;
  HTTPClient http;
  http.setTimeout(TIMEOUT_HTTP_MS);
  if (!http.begin(cliente, String(API_BASE_URL) + API_LEITURAS)) {
    Serial.println("[BACKEND] URL invalida");
    return;
  }
  http.addHeader("Content-Type", "application/json");
  int codigo = http.POST(json);
  Serial.printf("[BACKEND] POST %s -> %d\n", API_LEITURAS, codigo);
  http.end();
}

// =====================================================================================
//  CICLO DE LEITURA (60 min / 10 min)
// =====================================================================================
uint32_t intervaloAtual() {
  bool chuva = (leitura.p48Ok && leitura.p48 > 0) || (leitura.p24Ok && leitura.p24 > 0) ||
               (leitura.p72Injetado && leitura.p72 > 0);
  return (chuva || leitura.nivel != VERDE) ? INTERVALO_EVENTO_MS : INTERVALO_NORMAL_MS;
}

void imprimirLeitura() {
  Serial.println("--------------------------------------------------");
  if (leitura.sensorOk) Serial.printf("[SOLO]  ADC=%d | S=%.2f (S1=%.2f, S2=%.2f)\n", leitura.adc, leitura.S, S1, S2);
  else Serial.printf("[SOLO]  ADC=%d | HIGROMETRO INDISPONIVEL\n", leitura.adc);
  if (leitura.p72Injetado && chuvaCompleta()) Serial.printf("[CHUVA] P72=%.1f mm (INJETADO - TESTE)\n", leitura.p72);
  else if (chuvaCompleta()) Serial.printf("[CHUVA] P48=%.1f + P24=%.1f = P72=%.1f mm (P1=%.0f, P2=%.0f)\n",
                                          leitura.p48, leitura.p24, leitura.p72, P1_MM, P2_MM);
  else if (chuvaIncompleta()) {
    Serial.printf("[CHUVA] DADOS DE CHUVA INCOMPLETOS - P72 >= %.1f mm (minimo garantido)\n", p72Minimo());
    String p48t = !leitura.p48Ok ? String("indisponivel")
                : (leitura.p48DiasNulos > 0 ? String(leitura.p48, 1) + " mm, " + String(leitura.p48DiasNulos) + " dia(s) sem registro"
                                            : String(leitura.p48, 1) + " mm");
    String p24t = leitura.p24Ok ? String(leitura.p24, 1) + " mm" : String("indisponivel");
    Serial.println("[CHUVA] P48: " + p48t + " | P24: " + p24t);
    if (p72Minimo() < P1_MM) Serial.println("[CHUVA] Minimo abaixo de P1: tratado como sem dados de chuva");
    if (leitura.p72Injetado) Serial.println("[CHUVA] (INJETADO - TESTE: dia sem registro simulado)");
  } else Serial.println("[CHUVA] SEM DADOS DE CHUVA (P48 e P24 indisponiveis)");
  Serial.printf("[NIVEL] %s | buzzer %s\n", textoNivel(leitura.nivel), leitura.nivel == VERMELHO ? "LIGADO" : "desligado");
  Serial.printf("[CICLO] proximo em %lu s (modo demonstracao x%lu)\n", (unsigned long)(intervaloAtual() / 1000UL),
                (unsigned long)FATOR_DEMO);
}

void executarCiclo() {
  lerSaturacao();
  obterChuva();
  leitura.nivel = avaliarNivel(leitura);
  acionarAlertaLocal(leitura.nivel);
  imprimirLeitura();
  enviarLeitura();
  ultimoCicloMs = millis();
  cicloForcado = false;
}

// =====================================================================================
//  COMANDOS SERIAL (somente teste)
// =====================================================================================
void cmdHelp() {
  Serial.println("Comandos:");
  Serial.println("  status          - ultimo ciclo e leitura atual do higrometro");
  Serial.println("  calibrar        - mostra L_seco e L_sat");
  Serial.println("  calibrar seco   - grava a leitura atual como L_seco (sonda em solo seco)");
  Serial.println("  calibrar sat    - grava a leitura atual como L_sat (sonda em solo saturado)");
  Serial.println("  calibrar reset  - volta para a calibracao padrao");
  Serial.println("  chuva <mm>      - TESTE: injeta P72 em mm e roda um ciclo");
  Serial.println("  chuva parcial <mm> - TESTE: dados incompletos (1 dia sem registro); minimo = mm");
  Serial.println("  chuva na        - TESTE: simula APIs fora do ar (sem dados de chuva)");
  Serial.println("  chuva off       - volta a usar BNDMET + OpenWeatherMap");
  Serial.println("  help            - esta lista");
}

void cmdStatus() {
  imprimirLeitura(); // valores do último ciclo
  int adc = lerAdc();
  Serial.printf("[AGORA] ADC=%d | S=%.2f%s\n", adc, calcularSaturacao(adc), leituraValida(adc) ? "" : " (leitura invalida)");
  Serial.printf("[CALIB] L_seco=%d | L_sat=%d | modo chuva=%s\n", lSeco, lSat,
                modoChuva == 1 ? "injetado" : (modoChuva == 2 ? "sem dados" : (modoChuva == 3 ? "incompleto injetado" : "APIs")));
}

void cmdCalibrar(const String &arg) {
  int adc = lerAdc();
  int novoSeco = lSeco, novoSat = lSat;
  if (arg == "seco") novoSeco = adc;
  else if (arg == "sat") novoSat = adc;
  else if (arg == "reset") { novoSeco = L_SECO_PADRAO; novoSat = L_SAT_PADRAO; }
  if (arg.length()) {
    if (calibracaoValida(novoSeco, novoSat)) { lSeco = novoSeco; lSat = novoSat; salvarCalibracao(); }
    else Serial.println("[CALIB] Recusado: L_seco e L_sat ficariam iguais");
  }
  Serial.printf("[CALIB] L_seco=%d | L_sat=%d | ADC atual=%d\n", lSeco, lSat, adc);
}

void cmdChuva(const String &arg) {
  if (arg == "off") modoChuva = 0;
  else if (arg == "na") modoChuva = 2;
  else if (arg.startsWith("parcial ") && arg.length() > 8 && (isDigit(arg[8]) || arg[8] == '.')) {
    modoChuva = 3;
    p72Teste = arg.substring(8).toFloat();
  }
  else if (arg.length() && (isDigit(arg[0]) || arg[0] == '.')) { modoChuva = 1; p72Teste = arg.toFloat(); }
  else { Serial.println("Uso: chuva <mm> | chuva parcial <mm> | chuva na | chuva off"); return; }
  Serial.printf("[TESTE] modo chuva = %s\n", modoChuva == 1 ? "injetado" : (modoChuva == 2 ? "sem dados" : (modoChuva == 3 ? "incompleto injetado" : "APIs")));
  cicloForcado = true;
}

void tratarComando(String linha) {
  linha.trim();
  linha.toLowerCase();
  int esp = linha.indexOf(' ');
  String cmd = esp < 0 ? linha : linha.substring(0, esp);
  String arg = esp < 0 ? "" : linha.substring(esp + 1);
  arg.trim();
  if (cmd == "status") cmdStatus();
  else if (cmd == "calibrar") cmdCalibrar(arg);
  else if (cmd == "chuva") cmdChuva(arg);
  else if (cmd == "help") cmdHelp();
  else if (cmd.length()) Serial.println("Comando desconhecido. Digite help.");
}

void lerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (linhaSerial.length()) tratarComando(linhaSerial);
      linhaSerial = "";
    } else {
      linhaSerial += c;
    }
  }
}

// =====================================================================================
//  SETUP E LOOP
// =====================================================================================
void configurarPinos() {
  const uint8_t saidas[] = {PIN_LED_VERDE, PIN_LED_AMARELO, PIN_LED_VERMELHO, PIN_BUZZER};
  for (uint8_t p : saidas) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
}

void testarIndicadores() { // acende cada LED e toca o buzzer rapidamente na partida
  const uint8_t leds[] = {PIN_LED_VERDE, PIN_LED_AMARELO, PIN_LED_VERMELHO};
  for (uint8_t p : leds) { digitalWrite(p, HIGH); delay(300); digitalWrite(p, LOW); } // Fonte: software
  digitalWrite(PIN_BUZZER, HIGH); delay(150); digitalWrite(PIN_BUZZER, LOW);          // Fonte: software
}

void setup() {
  Serial.begin(BAUD);
  delay(500); // Fonte: software (estabiliza a serial)
  Serial.println("\n=== Alerta de barragem v21 (ESP8266) - PROTOTIPO FISICO ===");
  Serial.printf("Modo demonstracao: tempos divididos por %lu\n", (unsigned long)FATOR_DEMO);
  configurarPinos();
  testarIndicadores();
  carregarCalibracao();
  if (garantirWiFi()) sincronizarRelogio();
  cmdHelp();
}

// Entre um ciclo e outro (10 ou 60 min) o firmware fica em silêncio; esta linha mostra
// que o loop continua rodando e quanto falta para a próxima leitura.
void avisarEspera() {
  static uint32_t ultimoAviso = 0;
  if (INTERVALO_EVENTO_MS <= AVISO_ESPERA_MS) return; // ciclo curto (modo demonstração): sem aviso
  if (millis() - ultimoAviso < AVISO_ESPERA_MS) return;
  ultimoAviso = millis();
  uint32_t decorrido = millis() - ultimoCicloMs;
  uint32_t falta = intervaloAtual() > decorrido ? intervaloAtual() - decorrido : 0;
  Serial.printf("[ESPERA] proximo ciclo em %lu min %02lu s | ADC agora=%d (digite status ou chuva <mm>)\n",
                (unsigned long)(falta / 60000UL), (unsigned long)((falta / 1000UL) % 60UL), lerAdc());
}

void loop() {
  lerSerial();
  if (cicloForcado || millis() - ultimoCicloMs >= intervaloAtual()) executarCiclo();
  else avisarEspera();
  delay(20); // Fonte: software (libera o processador para o Wi-Fi do ESP8266)
}
