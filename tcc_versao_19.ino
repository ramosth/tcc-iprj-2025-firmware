/*
 * SISTEMA DE MONITORAMENTO DE BARRAGEM DE REJEITOS
 * Integração BNDMET (Base Nacional de Dados Meteorológicos) e ESP8266 NodeMCU
 * TCC - Engenharia da Computação / UERJ-IPRJ
 * Autora: Thamires Ramos dos Santos
 *
 * VERSÃO: 19
 *
 * CHANGELOG v19 (sobre v17):
 *   Opção C — flag simulacaoAtiva + contador de 3 envios:
 *     Resolve os três problemas identificados na sessão v4:
 *
 *   Problema 1 — recomendação perdia "[Simulação]" nos ciclos seguintes:
 *     gerarRecomendacaoDetalhada() agora retorna sem sobrescrever quando
 *     simulacaoAtiva=true, preservando o texto "[Simulação]" do início ao fim.
 *
 *   Problema 2 — riscoIntegrado era recalculado pelo loop após o 1º envio:
 *     analisarRiscoIntegrado() no bloco de intervalo de análise do loop agora
 *     é bloqueado quando simulacaoAtiva=true, preservando todos os valores
 *     setados pela simulação (risco, componentes, indiceRisco).
 *
 *   Problema 4 — buzzer disparava após alerta vermelho → alerta amarelo/verde:
 *     Ao encerrar a simulação (simulacaoAtiva=false), buzzerAtivo é forçado
 *     para false e noTone() é chamado, garantindo silêncio imediato.
 *
 *   Encerramento automático:
 *     simulacaoEnviosRestantes conta regressivamente a cada envio à API.
 *     Ao chegar a 0: simulacaoAtiva=false, modoManual=false, buzzerAtivo=false.
 *     Serial exibe "🔁 Simulação encerrada — retornando ao modo automático".
 *
 *   Ajustes v18 também incorporados:
 *     controlarSistemaFisico(): LEDs exibem umidade e risco em todos os níveis.
 *     testarCalculosTCC(): usa dados reais capturados (não valores fixos).
 *
 * VERSÃO: 17 (original)
 *
 * CHANGELOG v15 (sobre v14):
 *   INTERVALO_ENVIO_TESTE — nova constante (10s) que substitui os intervalos
 *     adaptativos de envio à API local durante coletas para o TCC.
 *     Para desativar e voltar aos intervalos de produção, comentar a linha
 *     "return INTERVALO_ENVIO_TESTE;" em obterIntervaloEnvioAPI().
 *
 *   INTERVALO_RETRY_API_FALHA — nova constante (30s) usada como intervalo
 *     de retry quando BNDMET ou OWM falham, em vez do intervalo normal
 *     adaptativo (que pode chegar a 5min no VERDE). Garante recuperação
 *     rápida após falha temporária das APIs externas.
 *
 *   Loop/BNDMET — retry acelerado: quando dadosBNDMET.apiDisponivel=false,
 *     o intervalo de reconexão passa de obterIntervaloBNDMET() para
 *     INTERVALO_RETRY_API_FALHA (30s), retornando ao normal após sucesso.
 *
 *   Loop/OWM — retry acelerado: quando dadosMeteo.timestamp=0 (OWM ainda
 *     não respondeu com sucesso), o intervalo passa de INTERVALO_OWM (30min)
 *     para INTERVALO_RETRY_API_FALHA (30s). A flag owmDisponivel é agora
 *     baseada no timestamp real dos dados, não na temperatura (flag frágil).
 *
 * CHANGELOG v14 (sobre v13):
 *   Fix DIV-1 — Loop/cooldown: recomendação atualizada com umidade atual a cada leitura.
 *              Na v13, durante os ciclos de cooldown (aguardando 3 leituras seguras),
 *              analiseRisco.recomendacao ficava congelada com o texto da última ruptura
 *              ativa. Agora a string é recalculada com dadosLocais.umidadeSolo a cada
 *              ciclo do bloco de cooldown no loop().
 *
 *   Fix DIV-3 — Loop/cooldown: v_taxa recalculado durante cooldown.
 *              Na v13, analiseRisco.vTaxaVariacao ficava congelado com o valor do
 *              ciclo anterior de ruptura ativa durante os ciclos de cooldown.
 *              Agora chama fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR no bloco
 *              de cooldown, alinhando com o comportamento de acionarRuptura().
 *
 * CHANGELOG v13 (sobre v12):
 *   Fix 1 — acionarRuptura(): chama calcularConfiabilidadeAnalise() ao final.
 *            Na v12, a função nunca era invocada no caminho de ruptura (só em
 *            analisarRiscoIntegrado()), fazendo confiabilidade ficar em 90% e
 *            estado_especial=null em todos os VERMELHO. Agora: confiabilidade=100%
 *            e estado_especial="RUPTURA" em todos os registros de ruptura.
 *
 *   Fix 2 — acionarRuptura(): recomendação atualizada a cada ciclo com umidade atual.
 *            Na v12, a string era gerada apenas no instante do acionamento e congelava
 *            com a umidade daquele momento. IDs de retorno (ex: umidade já em 2% mas
 *            texto dizia 49%) ficavam com mensagem desatualizada. Agora usa
 *            dadosLocais.umidadeSolo no momento de cada envio.
 *
 *   Fix 3 — Variável global aguardandoResetRuptura adicionada.
 *            Sinaliza quando o sistema está na janela de 3 leituras de cooldown após
 *            ruptura. Serializada em dadosBrutos no JSON → frontend pode exibir
 *            "Ruptura em desescalada — aguardando confirmação" em vez de RUPTURA puro.
 *
 *   Fix 4 — atualizarDadosPrecipitacao(): guarda NTP adicionada.
 *            BNDMET só é consultado se time(nullptr) > 1.000.000.000 (data válida).
 *            Evita requisições com datas 1969-12-31 nos primeiros ciclos antes da
 *            sincronização NTP, que retornavam qualidadeDados=0% sem dados reais.
 *
 *   Fix 5 — Campo bndmetInicializado adicionado ao JSON.
 *            Distingue "dados ainda não disponíveis" (antes da primeira consulta NTP)
 *            de "dados presentes mas com qualidade 0%" (falha real de estação).
 *            Frontend e banco podem filtrar esses registros iniciais corretamente.
 *
 * CHANGELOG v12 (sobre v11):
 *   Fix A — calcularTaxaVariacao(): normalização por 100.0f em vez de 10.0f.
 *            Variações extremas durante ruptura (±40–70%) contribuem proporcionalmente
 *            ao risco em vez de saturar o constrain em ±1,000.
 *
 *   Fix B — acionarRuptura(): recalcula vTaxaVariacao com fabsf(calcularTaxaVariacao())
 *            × PESO_TAXA_VAR. Na v11 o campo ficava congelado com o valor do ciclo
 *            AMARELO anterior (DIV-3 identificada no CSV leituras_sensor_10).
 *
 *   Fix C — calcularConfiabilidadeAnalise(): reformulação completa da semântica.
 *            Nova definição: grau de completude e atualidade dos dados do ciclo (0–100%).
 *            Tabela de descontos com pesos documentáveis no TCC:
 *              -40 sensor falha | -25 BNDMET fora | -10 qualidade BNDMET <80%
 *              -15 OWM fora | -10 WiFi | -10 buffer insuficiente (totalLeiturasSensor < 5)
 *            Substituição de indiceHistorico (circular → bug cíclico) por
 *            totalLeiturasSensor (contador não-circular desde o boot).
 *            RUPTURA → confiabilidade = 100% fixo (estado confirmado por hardware).
 *            Detalhes do cálculo (cada desconto individual) salvos em
 *            dadosBrutos.confiabilidade_detalhes no JSON → persiste em dados_brutos (jsonb)
 *            no PostgreSQL para auditoria pelo operador e frontend.
 *
 *   Fix D — ativarAlarmeBuzzer(): yield() adicionado após cada ciclo de beep.
 *            Libera o scheduler do ESP8266 durante períodos longos de buzzer ativo,
 *            prevenindo watchdog reset e aliviando pressão no freeHeap.
 *
 * CARACTERÍSTICAS:
 *    - Dados de umidade local (higrômetro ESP8266)
 *    - Dados meteorológicos via API BNDMET (estação D6594 — Alberto Flores)
 *    - Dados de precipitação histórica e atual
 *    - Armazenamento no backend Node.js / PostgreSQL + TimescaleDB
 *    - Sistema de decisão inteligente com equação de risco de 7 variáveis
 *    - Previsão de risco baseada em múltiplos fatores
 *    - Confiabilidade auditável por ciclo (detalhes persistidos no banco)
 *
 * Hardware:
 *    - ESP8266 NodeMCU
 *    - Higrômetro Eletrogate (pino A0)
 *    - LED Verde  (D1) | LED Amarelo (D2) | LED Vermelho (D3)
 *    - Buzzer (D4)
 *
 * APIs Utilizadas:
 *    - Externas:
 *      - BNDMET (precipitação histórica, dados meteorológicos oficiais)
 *        - api-bndmet.decea.mil.br (HTTPS)
 *        - I006 (diário)  → /v1/estacoes/D6594/fenomenos/I006  → precipitacao24h, 7d, 30d
 *        - I175 (horário) → /v1/estacoes/D6594/fenomenos/I175  → precipitacaoAtual (observação)
 *      - OpenWeatherMap (previsão e dados complementares)
 *        - OWM /weather  → HTTPS | grnd_level (917 hPa real) + rain.1h
 *        - OWM /forecast → HTTPS | cnt=8 blocos × 3h = 24h | rain.3h + pop
 *    - API própria:
 *      - backend Node.js (192.168.1.108:3001)
 *
 * Lógica do Sistema:
 *    30% = Linha de ruptura
 *    25% = Valor crítico (Fator_lençol = 1,0)
 *
 *  FÓRMULA DE RISCO v4 (pesos somam 1,00 — Equação 5 TCC expandida):
 *    V_lencol        = (umidade% / 25%) × 0,40
 *    V_ch_atual      = (precip24h / 50mm) × 0,08          ← BNDMET I006
 *    V_ch_historica  = (precip7d  / 150mm) × 0,12         ← BNDMET I006
 *    V_ch_mensal     = (precip30d / 300mm) × 0,10         ← BNDMET I006
 *    V_ch_futura     = Fator_discreto × 0,15              ← OWM /forecast cnt=8 (24h)
 *      Fator_discreto: Fraca(<5mm)=0,00 | Moderada(5–25mm)=0,25
 *                      Forte(25–50mm)=0,50 | Muito Forte(50–80mm)=0,75
 *                      Pancada(≥80mm)=1,00
 *    V_taxa_var      = (ΔU / 100%) × 0,10  ← Fix A v12: normalização por 100 (era 10)
 *    V_pressao       = (queda_hPa / 5hPa) × 0,05
 *    Amplificação    = × 1,2 se Fator_lençol ≥ 0,70 E previsão ≥ Moderada (≥5mm/24h)
 *
 *  Equação Final:
 *    Fator_risco = V_lencol + V_ch_atual + V_ch_historica + V_ch_mensal
 *                 + V_ch_futura + V_taxa_var + V_pressao
 *
 *  Faixas de alerta de risco (Tabela 5 TCC):
 *    0,00 – 0,50 → VERDE
 *    0,50 – 0,80 → AMARELO
 *    > 0,80      → VERMELHO + Buzzer
 *    ≥ 30% umid  → RUPTURA (fatorRisco = 1,0 imediato)
 *
 *  Comandos Serial:
 *    status | analise | debug | calibrar | reset | help | teste | enviar | api
 *    alerta verde | alerta amarelo | alerta vermelho (simulação de nível)
 *
 * ========================================================================================================================
 */

// ============================================================
//  BIBLIOTECAS
// ============================================================
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <time.h>

// ============================================================
//  CONFIGURAÇÕES
// ============================================================

// WiFi
#define WIFI_SSID "Ramos1"
#define WIFI_PASS "Senha@2026"

// API própria (backend Node.js)
#define API_BASE_URL "http://192.168.1.108:3001"
#define API_ENDPOINT "/api/sensor/dados"
#define API_STATUS_ENDPOINT "/api/sensor/status"
#define API_TOKEN ""

// BNDMET — api-bndmet.decea.mil.br (HTTPS)
// Endpoint: /v1/estacoes/{estacao}/fenomenos/{codigo}?dataInicio=...&dataFinal=...
// Header obrigatório: x-api-key
#define BNDMET_HOST "api-bndmet.decea.mil.br"
#define BNDMET_API_KEY "F9prvVKpaQ1qNtQywCN2sily029xgNaq"
#define BNDMET_ESTACAO "D6594"  // Alberto Flores (ANA) — substitui A555 (em pane)
#define BNDMET_COD_I006 "I006"  // fenômeno precipitação diária
#define BNDMET_COD_I175 "I175"  // fenômeno precipitação horária

// OpenWeatherMap — pro.openweathermap.org (HTTPS obrigatório)
#define OWM_HOST "pro.openweathermap.org"
#define OWM_API_KEY "04b8a531e11670b8099c49e16ba8f676"
#define OWM_LAT "-20.1433"  // Brumadinho — Córrego do Feijão
#define OWM_LON "-44.1997"

// ============================================================
//  PINOS
// ============================================================
#define PIN_HIGROMETRO A0    // Sensor de umidade (analógico)
#define PIN_LED_VERDE D1     // LED Verde - Seguro
#define PIN_LED_AMARELO D2   // LED Amarelo - Atenção
#define PIN_LED_VERMELHO D3  // LED Vermelho - Crítico
#define PIN_BUZZER D4        // Buzzer de alerta

// ============================================================
//  CALIBRAÇÃO DO SENSOR (persistida em EEPROM)
// ============================================================
int SENSOR_SECO = 1023;  // Valor ADC lido pelo higrômetro em solo completamente seco → 0% umidade (adimensional, range 0–1023)
int SENSOR_UMIDO = 240;  // Valor ADC lido pelo higrômetro em solo saturado em água → 100% umidade (adimensional, range 0–1023)

// ============================================================
//  LIMIARES — FONTES VERIFICADAS
// ============================================================
// Umidade
const float UMIDADE_CRITICA = 25.0f;  // Limiar de saturação crítica do solo (%) — Fator_lençol = 1,0 quando atingido (Equação 3 TCC)
const float UMIDADE_RUPTURA = 30.0f;  // Limiar de ruptura imediata (%) — força fatorRisco = 1,0 e aciona LED vermelho + buzzer independente da equação

// Precipitação — divisores de normalização para cada janela temporal
const float LIMIAR_24H = 50.0f;      // Precipitação acumulada 24h (mm) — referência: Merriespruit 1994 + AlertaRio nível 3 (Rico et al., 2008)
const float LIMIAR_7D = 150.0f;      // Precipitação acumulada 7 dias (mm) — referência: Rico et al. (2008), 147 rompimentos mundiais
const float LIMIAR_30D = 300.0f;     // Precipitação acumulada 30 dias (mm) — referência: análise histórica estação D6594, Brumadinho (média 328mm)
const float LIMIAR_24H_OWM = 80.0f;  // Acumulado previsto 24h (mm) correspondente à classe "Pancada de Chuva" — Tabela AlertaRio (Tabela 4 TCC)

// Pressão atmosférica
const float QUEDA_PRESSAO_ALERTA = 5.0f;  // Queda de pressão em 3h (hPa) que caracteriza instabilidade atmosférica iminente — divisor de normalização de V_pressão
const float PRESSAO_MIN = 700.0f;         // Pressão mínima válida (hPa) — cobre leituras grnd_level em altitudes elevadas (ex.: regiões montanhosas)
const float PRESSAO_MAX = 1100.0f;        // Pressão máxima válida (hPa) — limite superior para filtragem de leituras inválidas da OWM

// ============================================================
//  PESOS DA FÓRMULA (soma = 1,00)
// ============================================================
const float PESO_LENCOL = 0.40f;     // Peso de V_lençol — nível do lençol freático (maior peso por ser indicador direto de saturação do maciço)
const float PESO_CH_ATUAL = 0.08f;   // Peso de V_ch.24h — precipitação acumulada nas últimas 24h
const float PESO_CH_HIST = 0.12f;    // Peso de V_ch.7d  — precipitação acumulada nos últimos 7 dias
const float PESO_CH_MENSAL = 0.10f;  // Peso de V_ch.30d — precipitação acumulada nos últimos 30 dias
const float PESO_CH_FUTURA = 0.15f;  // Peso de V_ch.fut — previsão de intensidade pluviométrica nas próximas 24h
const float PESO_TAXA_VAR = 0.10f;   // Peso de V_taxa   — taxa de variação da umidade do solo (velocidade de saturação)
const float PESO_PRESSAO = 0.05f;    // Peso de V_pressão — queda de pressão atmosférica (indicador antecipatório)

// Amplificação: solo saturado (≥70% do crítico) + chuva futura alta (≥60%)
const float FATOR_AMPLIF = 1.20f;     // Coeficiente de amplificação aplicado quando solo saturado + previsão ≥ Moderada (adimensional)
const float LIMIAR_SOLO_SAT = 0.70f;  // Fator_lençol mínimo para ativar amplificação (adimensional) — equivale a umidade ≥ 17,5%
// Amplificação ativada quando previsão >= Moderada (chuvaFutura24h >= 5mm)

// Faixas de alerta (Tabela 5 TCC)
const float LIMIAR_VERDE = 0.45f;    // Fator de risco máximo para nível VERDE  — acima disso: AMARELO (adimensional, escala [0–1])
const float LIMIAR_AMARELO = 0.75f;  // Fator de risco máximo para nível AMARELO — acima disso: VERMELHO (adimensional, escala [0–1])

// ============================================================
//  BUFFERS E HISTÓRICO
// ============================================================
const int BUFFER_UMIDADE = 10;  // Capacidade do buffer circular de umidade — número de leituras armazenadas para cálculo da taxa de variação
const int HIST_PRESSAO = 6;     // Capacidade do histórico de pressão — 6 leituras × intervalo OWM (30min) = janela de 3h para cálculo de queda

// ============================================================
//  INTERVALOS ADAPTATIVOS (ms) — FASE DE TESTE
//  Produção recomendada: VERDE=15-30min sensor, API 1-2×/dia
// ============================================================
const unsigned long INTERVALO_SENSOR_VERDE = 30000UL;        // Intervalo de leitura do higrômetro em nível VERDE (ms) — 30s
const unsigned long INTERVALO_SENSOR_AMARELO = 10000UL;      // Intervalo de leitura do higrômetro em nível AMARELO (ms) — 10s
const unsigned long INTERVALO_SENSOR_VERMELHO = 5000UL;      // Intervalo de leitura do higrômetro em nível VERMELHO (ms) — 5s
const unsigned long INTERVALO_BNDMET_VERDE = 300000UL;       // Intervalo de consulta à API BNDMET em nível VERDE (ms) — 5min
const unsigned long INTERVALO_BNDMET_AMARELO = 120000UL;     // Intervalo de consulta à API BNDMET em nível AMARELO (ms) — 2min
const unsigned long INTERVALO_BNDMET_VERMELHO = 60000UL;     // Intervalo de consulta à API BNDMET em nível VERMELHO (ms) — 1min
const unsigned long INTERVALO_METEO = 600000UL;              // Intervalo de consulta ao OWM /weather (ms) — 10min
const unsigned long INTERVALO_OWM = 1800000UL;               // Intervalo de consulta ao OWM /forecast (ms) — 30min
const unsigned long INTERVALO_ENVIO_API_VERDE = 60000UL;     // VERDE    — envia a cada 60s
const unsigned long INTERVALO_ENVIO_API_AMARELO = 20000UL;   // AMARELO  — envia a cada 20s
const unsigned long INTERVALO_ENVIO_API_VERMELHO = 10000UL;  // VERMELHO — envia a cada 10s
const unsigned long INTERVALO_ANALISE = 10000UL;             // Intervalo de execução da análise de risco integrado (ms) — 10s

// ── Constantes de modo teste e retry ──────────────────────────────────────
// INTERVALO_ENVIO_TESTE: substitui os intervalos adaptativos de envio à API
// local durante sessões de coleta para o TCC.
// Em produção, comentar a linha abaixo para usar os intervalos adaptativos.
// const unsigned long INTERVALO_ENVIO_TESTE = 10000UL;  // 10s — todos os níveis (fase de teste)

// INTERVALO_RETRY_API_FALHA: intervalo reduzido de retry quando BNDMET ou
// OWM falham. Substitui os intervalos normais até a próxima consulta
// bem-sucedida, garantindo recuperação mais rápida sem sobrecarregar as APIs.
const unsigned long INTERVALO_RETRY_API_FALHA = 30000UL;  // 30s — retry após falha de API externa

// ============================================================
//  ESTRUTURAS DE DADOS
// ============================================================
struct DadosLocais {
  float umidadeSolo;        // Umidade do solo lida pelo higrômetro (%) — mapeada do ADC via SENSOR_SECO/SENSOR_UMIDO
  float fatorLocal;         // Fator_lençol normalizado [0–1] — umidadeSolo / UMIDADE_CRITICA (Equação 3 TCC)
  int valorADC;             // Leitura bruta do conversor analógico-digital do pino A0 (adimensional, range 0–1023)
  unsigned long timestamp;  // Instante da última leitura do sensor (ms desde o boot — millis())
  bool sensorOK;            // true quando o valor ADC está dentro do range esperado (190–1073), indicando sensor conectado
};

struct DadosBNDMET {
  float precipitacaoAtual;  // Precipitação horária mais próxima do horário atual via fenômeno I175 (mm) — observação pontual
  float precipitacao24h;    // Precipitação acumulada no dia anterior via fenômeno I006 (mm) — base para V_ch.24h
  float precipitacao7d;     // Precipitação acumulada nos últimos 7 dias via fenômeno I006 (mm) — base para V_ch.7d
  float precipitacao30d;    // Precipitação acumulada nos últimos 30 dias via fenômeno I006 (mm) — base para V_ch.30d
  String estacao;           // Código da estação meteorológica consultada (ex.: "D6594" — Alberto Flores, ANA)
  String statusAPI;         // Status da última requisição à API BNDMET: "OK" ou "FALHA"
  unsigned long timestamp;  // Instante da última consulta bem-sucedida à API BNDMET (ms desde o boot — millis())
  int qualidadeDados;       // Percentual de medições válidas (não nulas) retornadas pela API BNDMET (%)
  bool apiDisponivel;       // true quando a última requisição retornou HTTP 200 com estrutura JSON válida
};

struct DadosMeteorologicos {
  float temperatura;         // Temperatura do ar no local monitorado (°C) — campo "temp" do OWM /weather
  float umidadeExterna;      // Umidade relativa do ar (%) — campo "humidity" do OWM /weather
  float pressaoAtmosferica;  // Pressão atmosférica ao nível do solo (hPa) — campo "grnd_level" preferido, fallback "pressure" do OWM /weather
  float velocidadeVento;     // Velocidade do vento (m/s) — campo "wind.speed" do OWM /weather
  float chuvaAtualOWM;       // Precipitação na última hora (mm/h) — campo "rain.1h" do OWM /weather (ausente = 0, sem chuva no momento)
  String descricaoTempo;     // Descrição textual das condições meteorológicas atuais em pt_br (ex.: "chuva moderada")
  unsigned long timestamp;   // Instante da última consulta ao OWM /weather (ms desde o boot — millis())
};

struct PrevisaoTempo {
  float chuvaFutura24h;        // Precipitação total prevista para as próximas 24h (mm) — soma de rain.3h dos 8 blocos do OWM /forecast (cnt=8)
  String intensidadePrevisao;  // Classe de intensidade pluviométrica conforme Tabela 4 TCC: "Fraca" / "Moderada" / "Forte" / "Muito Forte" / "Pancada de Chuva"
  float fatorIntensidade;      // Fator discreto correspondente à classe de intensidade: 0,00 / 0,25 / 0,50 / 0,75 / 1,00 (adimensional)
  String tendencia;            // Tendência qualitativa da previsão (campo reservado para uso futuro)
  unsigned long timestamp;     // Instante da última consulta ao OWM /forecast (ms desde o boot — millis())
};

struct AnaliseRisco {
  float riscoIntegrado;  // Fator de risco final calculado pela Equação 5 TCC (adimensional, escala [0–1], podendo exceder 1,0 com amplificação)
  int indiceRisco;       // Fator de risco expresso em percentual inteiro [0–100] — riscoIntegrado × 100
  String nivelAlerta;    // Nível de alerta atual: "VERDE" | "AMARELO" | "VERMELHO"
  String statusTexto;    // Descrição textual do status: "SEGURO" | "ATENÇÃO" | "CRÍTICO" | "RUPTURA"
  String cor;            // Cor do LED ativo: "VERDE" | "AMARELO" | "VERMELHO"
  String recomendacao;   // Mensagem de recomendação operacional gerada em função do nível de risco
  int confiabilidade;    // Percentual de confiabilidade da análise (%) — reduzido por falhas de sensor, API indisponível ou dados desatualizados
  bool amplificado;      // true quando o mecanismo de amplificação (×1,20) foi aplicado neste ciclo

  // Componentes individuais da Equação 5 TCC expandida (adimensional, cada um já inclui o peso)
  float vLencol;          // V_lençol = Fator_lençol × 0,40
  float vChuvaAtual;      // V_ch.24h = (precip24h / 50mm) × 0,08
  float vChuvaHistorica;  // V_ch.7d  = (precip7d / 150mm) × 0,12
  float vChuvaMensal;     // V_ch.30d = (precip30d / 300mm) × 0,10
  float vChuvaFutura;     // V_ch.fut = fatorIntensidade × 0,15
  float vTaxaVariacao;    // V_taxa   = (ΔU / 10%) × 0,10
  float vPressao;         // V_pressão = (ΔP / 5hPa) × 0,05
  float vChuvaAcumulada;  // vChuvaAtual + vChuvaHistorica — campo de compatibilidade para o backend
};


// ============================================================
//  VARIÁVEIS GLOBAIS
// ============================================================
DadosLocais dadosLocais;         // Leituras do sensor higrômetro (umidade, ADC, fator)
DadosBNDMET dadosBNDMET;         // Dados de precipitação histórica via API BNDMET (mm)
DadosMeteorologicos dadosMeteo;  // Dados meteorológicos atuais via OWM /weather
PrevisaoTempo previsao;          // Previsão pluviométrica 24h via OWM /forecast
AnaliseRisco analiseRisco;       // Resultado da equação de risco integrado [0–1]

// Buffers circulares
float bufferUmidade[BUFFER_UMIDADE] = { 0 };  // Histórico de leituras de umidade do solo (%) para cálculo de taxa de variação
int bufferIndex = 0;                          // Índice atual de escrita no bufferUmidade [0–BUFFER_UMIDADE-1]
bool bufferCheio = false;                     // Indica se o bufferUmidade já completou ao menos um ciclo completo

float historicoPressao[HIST_PRESSAO] = { 0 };  // Histórico de leituras de pressão atmosférica (hPa) — janela de 3h
int indexPressao = 0;                          // Índice atual de escrita no historicoPressao
int totalLeiturasPressao = 0;                  // Total de leituras de pressão já registradas (máx. HIST_PRESSAO)

// Histórico de análise (últimas 10 leituras)
float historicoUmidade[10] = { 0 };       // Últimas 10 leituras de umidade do solo (%)
float historicoPrecipitacao[10] = { 0 };  // Últimas 10 leituras de precipitação 24h (mm)
float historicoRisco[10] = { 0 };         // Últimas 10 leituras do fator de risco integrado [0–1]
int indiceHistorico = 0;                  // Índice circular atual para os arrays de histórico [0–9]

// Controle de estado
bool wifiConectado = false;        // true quando conexão Wi-Fi está estabelecida
bool apiConectada = false;         // true quando o backend local responde com sucesso
bool sistemaInicializado = false;  // true após conclusão do setup() completo
bool modoManual = false;           // true quando operação manual via Serial está ativa (suspende atualizações automáticas)
bool simulacaoAtiva = false;       // true enquanto uma simulação de nível está em andamento (bloqueia recálculo e sobrescrita)
int simulacaoEnviosRestantes = 0;  // contador regressivo: ao chegar a 0 encerra simulacaoAtiva e modoManual automaticamente
bool buzzerAtivo = false;          // true quando o buzzer deve soar (nível de alerta vermelho ou ruptura)
int statusSistema = 0;             // Nível de alerta atual: 0=Verde | 1=Amarelo | 2=Vermelho
int tentativasEnvioAPI = 0;        // Contador de falhas consecutivas no envio para o backend
int tentativasReconexao = 0;       // Contador de tentativas de reconexão Wi-Fi sem sucesso

// Timestamps de controle (ms desde o boot — retorno de millis())
unsigned long ultimaLeituraSensor = 0;  // Último instante em que o higrômetro foi lido (ms)
unsigned long ultimaLeituraBNDMET = 0;  // Último instante em que a API BNDMET foi consultada (ms)
unsigned long ultimaLeituraMeteo = 0;   // Último instante em que OWM /weather foi consultado (ms)
unsigned long ultimaOWM = 0;            // Último instante em que OWM /forecast foi consultado (ms)
unsigned long ultimaAnalise = 0;        // Último instante em que a análise de risco foi executada (ms)
unsigned long ultimoEnvioAPI = 0;       // Último instante em que dados foram enviados ao backend (ms)

// FIX v12 (Fix C): contador total de leituras do sensor — não circular.
// Substitui indiceHistorico < 5 (que era circular e causava desconto cíclico na confiabilidade).
int totalLeiturasSensor = 0;  // Total acumulado de leituras do higrômetro desde o boot

// FIX v13 (Fix 3): sinaliza janela de cooldown após ruptura (3 leituras consecutivas seguras).
// Serializado em dadosBrutos → frontend exibe "Ruptura em desescalada" em vez de RUPTURA puro.
bool aguardandoResetRuptura = false;  // true enquanto contagemRetornoRuptura > 0 e < 3

// FIX v13 (Fix 5): distingue "BNDMET ainda não consultado" de "qualidade=0% por falha real".
// Evita falso alarme de qualidade nos registros iniciais antes da sincronização NTP.
bool bndmetInicializado = false;  // true após primeira consulta BNDMET bem-sucedida com NTP válido

// Clientes HTTP — instâncias globais reutilizáveis
WiFiClient wifiClient;              // Cliente HTTP sem TLS — usado para o backend local (HTTP)
WiFiClientSecure wifiClientSecure;  // Cliente HTTP com TLS — usado para BNDMET e OWM (HTTPS)

// ============================================================
//  ESTRUTURA AUXILIAR — DETALHES DA CONFIABILIDADE (v12)
// ============================================================
struct ConfiabilidadeDetalhes {
  int descontoSensor;
  int descontoBndmetFora;
  int descontoQualidadeBndmet;
  int descontoOWM;
  int descontoWifi;
  int descontoBuffer;
  int totalDesconto;
  int resultado;
  bool estadoEspecialRuptura;
};
ConfiabilidadeDetalhes detalhesConfiab;  // preenchido a cada ciclo de calcularConfiabilidadeAnalise()

// ============================================================
//  PROTÓTIPOS (forward declarations)
// ============================================================
void analisarRiscoIntegrado();
bool enviarDadosParaAPI();
String criarPayloadJSON();
String obterTimestampISO();
String formatarTempo(unsigned long ts);
void mostrarStatusConectividade();
void mostrarComandosDisponiveis();
void mostrarAnaliseDetalhada();
void recalibrarSensor();
void resetarSistema();
void executarTesteCompleto();
void debugConectividade();
void testarCalculosTCC();
bool verificarStatusAPI();
void verificarConectividade();
void gerarRecomendacaoDetalhada();
void calcularConfiabilidadeAnalise();
void atualizarHistoricoAnalise();
void controlarSistemaFisico();
void ativarAlarmeBuzzer();
void aplicarLimitesAlerta();


// ============================================================
// ============================================================
//  FUNÇÕES DE CÁLCULO — EQUAÇÕES DO TCC
// ============================================================
// ============================================================

// Equação 3 TCC: Fator_lençol = umidade% / UMIDADE_CRITICA
float calcularFatorLencolFreatico(float umidade) {
  return constrain(umidade / UMIDADE_CRITICA, 0.0f, 1.0f);
}

// Equação 4 TCC: V_lençol = Fator_lençol × 0,40
float calcularVLencolFreatico(float umidade) {
  return calcularFatorLencolFreatico(umidade) * PESO_LENCOL;
}

// V_ch.atual = (precip24h / LIMIAR_24H) × 0,08
float calcularVChuvaAtual(float precip24h) {
  return constrain(precip24h / LIMIAR_24H, 0.0f, 1.0f) * PESO_CH_ATUAL;
}

// V_ch.histórica = (precip7d / LIMIAR_7D) × 0,12
float calcularVChuvaHistorica(float precip7d) {
  return constrain(precip7d / LIMIAR_7D, 0.0f, 1.0f) * PESO_CH_HIST;
}

// V_ch.mensal = (precip30d / LIMIAR_30D) × 0,10
float calcularVChuvaMensal(float precip30d) {
  return constrain(precip30d / LIMIAR_30D, 0.0f, 1.0f) * PESO_CH_MENSAL;
}

// V_ch.futura = Fator_discreto × 0,15
// Fator discreto conforme Tabela AlertaRio (Tabela 4 TCC):
//   Fraca (<5mm)=0,00 | Moderada (5–25mm)=0,25 | Forte (25–50mm)=0,50
//   Muito Forte (50–80mm)=0,75 | Pancada (≥80mm)=1,00
float calcularVChuvaFutura(float forecast24h) {
  float fator = calcularFatorPrevisaoIntensidade(obterIntensidadePrevisao());
  return fator * PESO_CH_FUTURA;
}

// Taxa de variação de umidade — buffer circular (bidirecional)
// Retorna valor no intervalo [-1,0 ; +1,0]:
//   positivo → solo absorvendo água (risco crescente)
//   negativo → solo drenando     (pode indicar dreno súbito — igualmente relevante)
// O componente V_taxa_variacao usa abs() para contribuir ao risco em ambos os sentidos.
float calcularTaxaVariacao() {
  int total = bufferCheio ? BUFFER_UMIDADE : bufferIndex;
  if (total < 2) return 0.0f;
  int idxAntigo = bufferCheio ? bufferIndex : 0;
  int idxNovo = (bufferIndex - 1 + BUFFER_UMIDADE) % BUFFER_UMIDADE;
  float variacao = bufferUmidade[idxNovo] - bufferUmidade[idxAntigo];
  // FIX v12 (Fix A): normaliza pelo range real do sensor (0–100%) em vez de 10%/leitura.
  // Motivação: variação máxima esperada entre a leitura mais antiga e a mais nova no buffer
  // pode atingir ±40–70% durante ruptura, saturando o constrain em ±1 com a divisão por 10.
  // Com divisão por 100, variações extremas contribuem proporcionalmente ao risco
  // (ex.: ΔU=40% → taxa=0,40 em vez de 1,00 travado), preservando a escala [-1 ; +1].
  return constrain(variacao / 100.0f, -1.0f, 1.0f);
}

// Queda de pressão em 3h — histórico circular
// Lógica: leitura_antiga - leitura_nova → positivo = pressão caindo
float calcularQuedaPressao() {
  if (totalLeiturasPressao < 2) return 0.0f;

  int idxAntigo, idxNovo;
  if (totalLeiturasPressao < HIST_PRESSAO) {
    idxAntigo = 0;
    idxNovo = totalLeiturasPressao - 1;
  } else {
    idxAntigo = indexPressao % HIST_PRESSAO;
    idxNovo = (indexPressao - 1 + HIST_PRESSAO) % HIST_PRESSAO;
  }

  if (historicoPressao[idxAntigo] < 1.0f) return 0.0f;
  float queda = historicoPressao[idxAntigo] - historicoPressao[idxNovo];
  Serial.printf("[Pressão] ant=%.2f nov=%.2f queda=%.2f hPa\n",
                historicoPressao[idxAntigo], historicoPressao[idxNovo], queda);
  return max(queda, 0.0f);
}

// Classificação de intensidade conforme Tabela 4 TCC (Sistema AlertaRio — janela 24h)
String obterIntensidadePrevisao() {
  float p = previsao.chuvaFutura24h;
  if (p >= 80.0f) return "Pancada de Chuva";
  if (p >= 50.0f) return "Muito Forte";
  if (p >= 25.0f) return "Forte";
  if (p >= 5.0f) return "Moderada";
  return "Fraca";
}

// Fator discreto por faixa de intensidade (Tabela 4 TCC)
float calcularFatorPrevisaoIntensidade(String intensidade) {
  if (intensidade == "Pancada de Chuva") return 1.00f;
  if (intensidade == "Muito Forte") return 0.75f;
  if (intensidade == "Forte") return 0.50f;
  if (intensidade == "Moderada") return 0.25f;
  return 0.0f;  // Fraca
}

// ============================================================
//  ANÁLISE INTEGRADA DE RISCO (Equação 5 TCC expandida)
// ============================================================
void analisarRiscoIntegrado() {
  Serial.println(F("🔍 ========== ANÁLISE DE RISCO INTEGRADO =========="));

  // ---- Componentes ----
  analiseRisco.vLencol = calcularVLencolFreatico(dadosLocais.umidadeSolo);
  analiseRisco.vChuvaAtual = calcularVChuvaAtual(dadosBNDMET.precipitacao24h);
  analiseRisco.vChuvaHistorica = calcularVChuvaHistorica(dadosBNDMET.precipitacao7d);
  analiseRisco.vChuvaMensal = calcularVChuvaMensal(dadosBNDMET.precipitacao30d);
  analiseRisco.vChuvaFutura = calcularVChuvaFutura(previsao.chuvaFutura24h);
  analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;
  analiseRisco.vPressao = constrain(calcularQuedaPressao() / QUEDA_PRESSAO_ALERTA,
                                    0.0f, 1.0f)
                          * PESO_PRESSAO;

  // Compatibilidade
  analiseRisco.vChuvaAcumulada = analiseRisco.vChuvaAtual + analiseRisco.vChuvaHistorica;

  // ---- Soma ----
  float soma = analiseRisco.vLencol
               + analiseRisco.vChuvaAtual
               + analiseRisco.vChuvaHistorica
               + analiseRisco.vChuvaMensal
               + analiseRisco.vChuvaFutura
               + analiseRisco.vTaxaVariacao
               + analiseRisco.vPressao;

  // ---- Amplificação ----
  // Ativada quando: Fator_lençol >= 0,70 E previsão >= Moderada (>=5mm/24h)
  float fatorLencol = calcularFatorLencolFreatico(dadosLocais.umidadeSolo);
  bool previsaoModeradaOuSuperior = (previsao.chuvaFutura24h >= 5.0f);
  analiseRisco.amplificado = (fatorLencol >= LIMIAR_SOLO_SAT && previsaoModeradaOuSuperior);
  if (analiseRisco.amplificado) {
    soma *= FATOR_AMPLIF;
    Serial.println(F("  ⚠️ AMPLIFICAÇÃO 1,2× aplicada (solo saturado + previsão ≥ Moderada)"));
  }

  // ---- Resultado final ----
  analiseRisco.riscoIntegrado = constrain(soma, 0.0f, 1.0f);
  analiseRisco.indiceRisco = (int)roundf(analiseRisco.riscoIntegrado * 100.0f);

  // ---- Log detalhado ----
  Serial.println(F("📊 Componentes (Equação 5 TCC expandida):"));
  Serial.printf("  V_lençol      = %.3f  (fator: %.3f × peso: 0,40)\n",
                analiseRisco.vLencol, fatorLencol);
  Serial.printf("  V_ch.atual    = %.3f  (precip24h=%.2fmm × 0,08)\n",
                analiseRisco.vChuvaAtual, dadosBNDMET.precipitacao24h);
  Serial.printf("  V_ch.histórica= %.3f  (precip7d=%.2fmm × 0,12)\n",
                analiseRisco.vChuvaHistorica, dadosBNDMET.precipitacao7d);
  Serial.printf("  V_ch.mensal   = %.3f  (precip30d=%.2fmm × 0,10)\n",
                analiseRisco.vChuvaMensal, dadosBNDMET.precipitacao30d);
  Serial.printf("  V_ch.futura   = %.3f  (forecast24h=%.2fmm, intensidade=%s, fator=%.2f × 0,15)\n",
                analiseRisco.vChuvaFutura, previsao.chuvaFutura24h,
                previsao.intensidadePrevisao.c_str(),
                calcularFatorPrevisaoIntensidade(previsao.intensidadePrevisao));
  Serial.printf("  V_taxa_var    = %.3f  (variação=%.2f%% | taxa=%.3f)\n",
                analiseRisco.vTaxaVariacao, calcularTaxaVariacao() * 100.0f, calcularTaxaVariacao());
  Serial.printf("  V_pressao     = %.3f  (queda=%.2fhPa × 0,05)\n",
                analiseRisco.vPressao, calcularQuedaPressao());
  Serial.printf("  FATOR_RISCO TOTAL = %.3f | ÍNDICE = %d%%\n",
                analiseRisco.riscoIntegrado, analiseRisco.indiceRisco);

  aplicarLimitesAlerta();
  calcularConfiabilidadeAnalise();
  atualizarHistoricoAnalise();
  gerarRecomendacaoDetalhada();
  controlarSistemaFisico();
}

// Aplicar limites conforme Tabela 5 TCC
void aplicarLimitesAlerta() {
  if (analiseRisco.riscoIntegrado <= LIMIAR_VERDE) {
    statusSistema = 0;
    analiseRisco.statusTexto = "SEGURO";
    analiseRisco.cor = "VERDE";
    analiseRisco.nivelAlerta = "VERDE";
  } else if (analiseRisco.riscoIntegrado <= LIMIAR_AMARELO) {
    statusSistema = 1;
    analiseRisco.statusTexto = "ATENÇÃO";
    analiseRisco.cor = "AMARELO";
    analiseRisco.nivelAlerta = "AMARELO";
  } else {
    statusSistema = 2;
    analiseRisco.statusTexto = "CRÍTICO";
    analiseRisco.cor = "VERMELHO";
    analiseRisco.nivelAlerta = "VERMELHO";
  }
  Serial.printf("🚨 STATUS: %s | Fator: %.3f\n",
                analiseRisco.statusTexto.c_str(), analiseRisco.riscoIntegrado);
}

// ============================================================
//  OVERRIDE DE RUPTURA
//  Força fatorRisco=1,0 e aciona LED vermelho + buzzer imediatamente.
// ============================================================
void acionarRuptura() {
  analiseRisco.riscoIntegrado = 1.0f;
  analiseRisco.indiceRisco = 100;
  analiseRisco.nivelAlerta = "VERMELHO";
  analiseRisco.statusTexto = "RUPTURA";
  analiseRisco.cor = "VERMELHO";
  // FIX v13 (Fix 2): recomendação usa dadosLocais.umidadeSolo no momento de CADA chamada,
  // não apenas no primeiro acionamento. Na v12 a string congelava com a umidade do instante
  // inicial — registros de retorno (umidade já baixa) ficavam com mensagem desatualizada.
  analiseRisco.recomendacao = "🚨 RUPTURA — EVACUAÇÃO IMEDIATA! Umidade acima da linha crítica ("
                              + String(dadosLocais.umidadeSolo, 1) + "% >= " + String(UMIDADE_RUPTURA, 0) + "%)";
  analiseRisco.amplificado = false;  // ruptura é direta, sem amplificação aplicável
  analiseRisco.vLencol = 0.40f;
  analiseRisco.vChuvaAtual = analiseRisco.vChuvaAtual;  // mantém valor atual
  // FIX v12 (Fix B): recalcula vTaxaVariacao com os dados reais do buffer durante a ruptura.
  // Na v11, este campo ficava congelado com o valor do ciclo AMARELO anterior (DIV-3).
  analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;
  statusSistema = 2;
  buzzerAtivo = true;

  Serial.println(F("\n🔴 ========================================"));
  Serial.println(F("🔴 ⛔ RUPTURA — UMIDADE ACIMA DO LIMIAR!"));
  Serial.printf("🔴 Umidade: %.2f%% | Limiar: %.2f%%\n",
                dadosLocais.umidadeSolo, UMIDADE_RUPTURA);
  Serial.println(F("🔴 ========================================\n"));

  // FIX v13 (Fix 1): calcular confiabilidade dentro de acionarRuptura().
  // Na v12, calcularConfiabilidadeAnalise() só era chamada via analisarRiscoIntegrado(),
  // que é bypassada durante ruptura — todos os VERMELHO ficavam com confiabilidade=90%
  // e estado_especial=null. Agora: confiabilidade=100 e estado_especial="RUPTURA".
  calcularConfiabilidadeAnalise();

  // Acionar hardware imediatamente — não aguardar próximo ciclo
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, HIGH);
  tone(PIN_BUZZER, 2400);
}

// ============================================================
//  HARDWARE — LEDs e Buzzer
// ============================================================
void controlarSistemaFisico() {
  if (modoManual) {
    Serial.println(F("⚠️ Modo manual — hardware não alterado"));
    return;
  }
  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA) return;  // ruptura já controlou

  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  buzzerAtivo = false;

  switch (statusSistema) {
    case 0:
      digitalWrite(PIN_LED_VERDE, HIGH);
      Serial.printf("🟢 LED VERDE — Situação SEGURA | Umidade: %.2f%% | Risco: %d%%\n", dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      break;
    case 1:
      digitalWrite(PIN_LED_AMARELO, HIGH);
      Serial.printf("🟡 LED AMARELO — Situação de ATENÇÃO | Umidade: %.2f%% | Risco: %d%%\n", dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      break;
    case 2:
      digitalWrite(PIN_LED_VERMELHO, HIGH);
      buzzerAtivo = true;
      Serial.printf("🔴 LED VERMELHO — Situação CRÍTICA | Umidade: %.2f%% | Risco: %d%%\n", dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      break;
  }
}

// Buzzer intermitente com frequência variável por nível de risco
void ativarAlarmeBuzzer() {
  static unsigned long ultimoBeep = 0;
  static bool estadoBuzzer = false;

  if (!buzzerAtivo) {
    noTone(PIN_BUZZER);
    estadoBuzzer = false;
    return;
  }

  unsigned long intervalo = 1000UL;
  if (analiseRisco.indiceRisco >= 90) intervalo = 300UL;
  else if (analiseRisco.indiceRisco >= 85) intervalo = 500UL;
  else if (analiseRisco.indiceRisco >= 80) intervalo = 800UL;

  if (millis() - ultimoBeep >= intervalo) {
    estadoBuzzer = !estadoBuzzer;
    digitalWrite(PIN_BUZZER, estadoBuzzer);
    ultimoBeep = millis();
    if (estadoBuzzer) {
      unsigned int freq = 2000;
      if (analiseRisco.indiceRisco >= 90) freq = 2400;
      else if (analiseRisco.indiceRisco >= 85) freq = 2200;
      else if (analiseRisco.indiceRisco >= 80) freq = 2000;
      tone(PIN_BUZZER, freq, intervalo / 2);
      ultimoBeep = millis();
      Serial.printf("🔊 ALARME — Risco: %d%% | Freq: %dHz\n",
                    analiseRisco.indiceRisco, freq);
    }
    yield();  // FIX v12 (Fix D): libera o scheduler do ESP8266 durante loop de buzzer
              // Evita watchdog reset e libera heap quando buzzerAtivo=true por períodos longos
  }
}

// ============================================================
//  SENSOR DE UMIDADE
// ============================================================
void atualizarDadosLocais() {
  if (modoManual) return;

  dadosLocais.valorADC = analogRead(PIN_HIGROMETRO);
  dadosLocais.umidadeSolo = constrain(
    (float)map(dadosLocais.valorADC, SENSOR_SECO, SENSOR_UMIDO, 0, 100),
    0.0f, 100.0f);
  dadosLocais.fatorLocal = calcularFatorLencolFreatico(dadosLocais.umidadeSolo);
  dadosLocais.sensorOK = (dadosLocais.valorADC > 0 && dadosLocais.valorADC < 1023);
  dadosLocais.timestamp = millis();

  // Atualizar buffer circular para taxa de variação
  bufferUmidade[bufferIndex] = dadosLocais.umidadeSolo;
  bufferIndex = (bufferIndex + 1) % BUFFER_UMIDADE;
  if (bufferIndex == 0) bufferCheio = true;

  // FIX v12 (Fix C): incrementa contador total não-circular — usado em calcularConfiabilidadeAnalise()
  totalLeiturasSensor++;

  // Atualizar histórico de análise
  historicoUmidade[indiceHistorico] = dadosLocais.umidadeSolo;

  Serial.printf("📊 Sensor — Umidade: %.2f%% | ADC: %d | Fator: %.3f | %s\n",
                dadosLocais.umidadeSolo, dadosLocais.valorADC,
                dadosLocais.fatorLocal,
                dadosLocais.sensorOK ? "OK" : "⚠️ FALHA");
}

// ============================================================
//  BNDMET — I006 (diário) + I175 (horário)
// ============================================================
void atualizarDadosPrecipitacao() {
  if (!wifiConectado) {
    Serial.println(F("❌ WiFi desconectado — não é possível consultar BNDMET"));
    return;
  }

  // FIX v13 (Fix 4): só consulta BNDMET se o NTP já sincronizou (data > ano 2001).
  // Sem esta guarda, nos primeiros ciclos após o boot time(nullptr) retorna valores
  // próximos de 0 → datas geradas como "1969-12-31" → API retorna 0 registros →
  // qualidadeDados=0% nos registros iniciais sem refletir falha real da estação.
  if (time(nullptr) < 1000000000UL) {
    Serial.println(F("⏳ NTP ainda não sincronizado — consulta BNDMET adiada"));
    return;
  }

  Serial.println(F("🌧 Consultando BNDMET..."));
  garantirWiFi();
  buscarDadosDiarios_I006();
  buscarDadosHorarios_I175();

  // Marcar como inicializado após primeira consulta bem-sucedida
  if (dadosBNDMET.apiDisponivel) bndmetInicializado = true;

  // Atualizar histórico
  historicoPrecipitacao[indiceHistorico] = dadosBNDMET.precipitacao24h;
  dadosBNDMET.timestamp = millis();
  dadosBNDMET.estacao = BNDMET_ESTACAO;

  Serial.printf("✓ BNDMET — 24h=%.2fmm | 7d=%.2fmm | 30d=%.2fmm | Atual=%.2fmm\n",
                dadosBNDMET.precipitacao24h, dadosBNDMET.precipitacao7d,
                dadosBNDMET.precipitacao30d, dadosBNDMET.precipitacaoAtual);

  if (dadosBNDMET.precipitacao24h > LIMIAR_24H)
    Serial.printf("  ⚠️ Precipitação 24h acima do limiar (%.0fmm)\n", LIMIAR_24H);
  if (dadosBNDMET.precipitacao7d > LIMIAR_7D)
    Serial.printf("  ⚠️ Precipitação 7d acima do limiar (%.0fmm)\n", LIMIAR_7D);
}

// I006 — diário (HTTPS): uma chamada com 30 dias, calcula 24h/7d/30d no parsing
// URL real: https://api-bndmet.decea.mil.br/v1/estacoes/D6594/fenomenos/I006?dataInicio=...&dataFinal=...
// Header: x-api-key (não "token")
// JSON retornado: { data: { data: [[timestamp_ms, valor], ...], nome, unidade } }
void buscarDadosDiarios_I006() {
  time_t agora = time(nullptr);
  char dtIni[11], dtFim[11], dtOntem[11];
  struct tm* t;
  time_t t30d = agora - 30L * 86400L;
  time_t t1d = agora - 86400L;
  t = localtime(&t30d);
  strftime(dtIni, sizeof(dtIni), "%Y-%m-%d", t);
  t = localtime(&t1d);
  strftime(dtFim, sizeof(dtFim), "%Y-%m-%d", t);
  strftime(dtOntem, sizeof(dtOntem), "%Y-%m-%d", t);

  // Endpoint
  String url = String("https://") + BNDMET_HOST
               + "/v1/estacoes/" + BNDMET_ESTACAO
               + "/fenomenos/" + BNDMET_COD_I006
               + "?dataInicio=" + dtIni
               + "&dataFinal=" + dtFim;

  Serial.printf("[I006] GET %s\n", url.c_str());

  // HTTPS obrigatório — setInsecure() evita verificação de certificado no ESP8266
  wifiClientSecure.setInsecure();
  HTTPClient http;
  http.begin(wifiClientSecure, url);
  http.addHeader("Accept", "*/*");
  http.addHeader("x-api-key", BNDMET_API_KEY);
  http.setTimeout(30000);

  int code = http.GET();
  Serial.printf("[I006] HTTP %d\n", code);

  if (code != HTTP_CODE_OK) {
    Serial.printf("[I006] Erro HTTP %d — mantendo valores anteriores\n", code);
    if (code > 0) Serial.println("[I006] Resposta: " + http.getString().substring(0, 200));
    dadosBNDMET.apiDisponivel = false;
    dadosBNDMET.statusAPI = "FALHA";
    dadosBNDMET.qualidadeDados = 0;
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();
  Serial.printf("[I006] Resposta: %d bytes\n", payload.length());

  // Buffer maior para resposta BNDMET (30 dias × N medições)
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    Serial.println(F("[I006] Erro ao parsear JSON"));
    dadosBNDMET.apiDisponivel = false;
    return;
  }

  // Estrutura real: doc["data"]["data"] = array de [timestamp_ms, valor]
  if (doc["data"]["data"].isNull() || !doc["data"]["data"].is<JsonArray>()) {
    Serial.println(F("[I006] Estrutura inesperada — campo data.data ausente"));
    Serial.println("[I006] Preview: " + payload.substring(0, 300));
    dadosBNDMET.apiDisponivel = false;
    return;
  }

  JsonArray dataArr = doc["data"]["data"].as<JsonArray>();
  int total = dataArr.size();
  int medicoesValidas = 0, medicoesNulas = 0;

  // Metadados da resposta (log informativo)
  Serial.printf("[I006] Parâmetro: %s | Unidade: %s | Periodicidade: %s | %d registros\n",
                doc["data"]["nome"] | "N/A",
                doc["data"]["unidade"] | "N/A",
                doc["data"]["periodicidade"] | "N/A",
                total);

  float soma7d = 0.0f, soma30d = 0.0f;
  float val24h = 0.0f;
  bool achou24h = false;

  // Cada elemento é [timestamp_ms, valor_ou_null]
  // A API retorna em ordem cronológica — os últimos elementos são os mais recentes
  for (int i = 0; i < total; i++) {
    JsonArray medicao = dataArr[i];
    if (medicao.size() < 2) continue;

    long long tsMed_ms = medicao[0].as<long long>();
    if (tsMed_ms <= 0) {
      medicoesNulas++;
      continue;
    }
    time_t tSec = (time_t)(tsMed_ms / 1000LL);
    struct tm* tmMed = localtime(&tSec);
    char dtMed[11];
    strftime(dtMed, sizeof(dtMed), "%Y-%m-%d", tmMed);

    bool nulo = medicao[1].isNull();
    float chuva = nulo ? 0.0f : medicao[1].as<float>();

    if (nulo) {
      medicoesNulas++;
      continue;
    }
    medicoesValidas++;

    // Identifica ontem (precipitacao24h)
    if (strcmp(dtMed, dtOntem) == 0) {
      val24h = chuva;
      achou24h = true;
    }

    // Últimos 7 dias (contando do fim do array)
    int diasDoFim = total - 1 - i;
    if (diasDoFim < 7) soma7d += chuva;
    soma30d += chuva;

    if (chuva > 0.0f)
      Serial.printf("  ☔ %s | %.2fmm\n", dtMed, chuva);
  }

  dadosBNDMET.precipitacao24h = achou24h ? val24h : 0.0f;
  dadosBNDMET.precipitacao7d = soma7d;
  dadosBNDMET.precipitacao30d = soma30d;
  dadosBNDMET.apiDisponivel = true;
  dadosBNDMET.statusAPI = "OK";
  dadosBNDMET.qualidadeDados = total > 0 ? (medicoesValidas * 100) / total : 0;

  Serial.printf("[I006] ✓ 24h=%.2f | 7d=%.2f | 30d=%.2f mm | Qualidade=%d%%\n",
                dadosBNDMET.precipitacao24h, dadosBNDMET.precipitacao7d,
                dadosBNDMET.precipitacao30d, dadosBNDMET.qualidadeDados);
  Serial.printf("[I006] Medições válidas: %d | Nulas: %d\n", medicoesValidas, medicoesNulas);
}

// I175 — horário (HTTPS): seleciona medição mais próxima do horário atual
// URL real: https://api-bndmet.decea.mil.br/v1/estacoes/D6594/fenomenos/I175?dataInicio=...&dataFinal=...
// JSON retornado: { data: { data: [[timestamp_ms, valor], ...] } }
void buscarDadosHorarios_I175() {
  time_t agora = time(nullptr);
  char dtHoje[11];
  struct tm* t = localtime(&agora);
  strftime(dtHoje, sizeof(dtHoje), "%Y-%m-%d", t);
  int horaAtualMin = t->tm_hour * 60 + t->tm_min;

  String url = String("https://") + BNDMET_HOST
               + "/v1/estacoes/" + BNDMET_ESTACAO
               + "/fenomenos/" + BNDMET_COD_I175
               + "?dataInicio=" + dtHoje
               + "&dataFinal=" + dtHoje;

  Serial.printf("[I175] GET %s\n", url.c_str());

  wifiClientSecure.setInsecure();
  HTTPClient http;
  http.begin(wifiClientSecure, url);
  http.addHeader("Accept", "*/*");
  http.addHeader("x-api-key", BNDMET_API_KEY);
  http.setTimeout(30000);

  int code = http.GET();
  Serial.printf("[I175] HTTP %d\n", code);

  if (code != HTTP_CODE_OK) {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    Serial.printf("[I175] Erro HTTP %d — precipitacaoAtual = 0\n", code);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();
  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    Serial.println(F("[I175] Erro ao parsear JSON"));
    return;
  }

  if (doc["data"]["data"].isNull() || !doc["data"]["data"].is<JsonArray>()) {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    Serial.println(F("[I175] Estrutura inesperada — campo data.data ausente"));
    return;
  }

  JsonArray dataArr = doc["data"]["data"].as<JsonArray>();
  if (dataArr.size() == 0) {
    dadosBNDMET.precipitacaoAtual = 0.0f;
    Serial.println(F("[I175] Nenhuma medição retornada"));
    return;
  }

  int menorDiff = 99999;
  float melhorVal = 0.0f;

  // Cada elemento é [timestamp_ms, valor_ou_null]
  for (JsonArray medicao : dataArr) {
    if (medicao.size() < 2) continue;

    long long tsMed_ms = medicao[0].as<long long>();
    if (tsMed_ms <= 0) continue;
    time_t tSec = (time_t)(tsMed_ms / 1000LL);
    struct tm* tmMed = localtime(&tSec);
    int medMin = tmMed->tm_hour * 60 + tmMed->tm_min;
    int diff = abs(horaAtualMin - medMin);

    if (!medicao[1].isNull() && diff < menorDiff) {
      menorDiff = diff;
      melhorVal = medicao[1].as<float>();
    }
  }

  dadosBNDMET.precipitacaoAtual = melhorVal;
  Serial.printf("[I175] ✓ Timestamp mais próximo (diff=%dmin): %.2fmm\n",
                menorDiff, dadosBNDMET.precipitacaoAtual);
}

// ============================================================
//  OPENWEATHERMAP — /weather + /forecast
// ============================================================
void atualizarDadosMeteorologicos() {
  if (!wifiConectado) {
    Serial.println(F("❌ WiFi desconectado — não é possível consultar OWM"));
    dadosMeteo.timestamp = 0;  // ← força indisponível
    return;
  }
  garantirWiFi();
  bool weatherOk = buscarWeatherAtual();  // precisa retornar bool
  bool forecastOk = buscarForecast();     // precisa retornar bool
  if (weatherOk || forecastOk) {
    dadosMeteo.timestamp = millis();  // só atualiza se pelo menos uma teve sucesso
  } else {
    dadosMeteo.timestamp = 0;  // força retry acelerado
  }
}

// /weather — pressão (grnd_level) + dados complementares + rain.1h
// HTTPS obrigatório conforme cURL real: https://pro.openweathermap.org/...
bool buscarWeatherAtual() {
  Serial.println(F("[OWM/weather] Buscando dados atuais..."));

  String url = String("https://") + OWM_HOST
               + "/data/2.5/weather?lat=" + OWM_LAT
               + "&lon=" + OWM_LON
               + "&appid=" + OWM_API_KEY
               + "&lang=pt_br&units=metric";

  Serial.printf("[OWM/weather] GET %s\n", url.c_str());

  wifiClientSecure.setInsecure();
  HTTPClient http;
  http.begin(wifiClientSecure, url);
  http.setTimeout(15000);
  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[OWM/weather] Erro HTTP %d\n", code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    Serial.println(F("[OWM/weather] Erro ao parsear JSON"));
    return false;
  }

  // Dados meteorológicos complementares
  dadosMeteo.temperatura = doc["main"]["temp"] | 0.0f;
  dadosMeteo.umidadeExterna = doc["main"]["humidity"] | 0.0f;
  dadosMeteo.velocidadeVento = doc["wind"]["speed"] | 0.0f;
  if (!doc["weather"][0]["description"].isNull())
    dadosMeteo.descricaoTempo = doc["weather"][0]["description"].as<String>();

  // Lógica: grnd_level preferido, fallback para pressure
  // grnd_level = pressão real no solo (mais preciso para locais em altitude)
  // pressure   = pressão ao nível do mar (menos preciso para o local específico)
  float pressaoNova = 0.0f;
  if (!doc["main"]["grnd_level"].isNull()) {
    pressaoNova = doc["main"]["grnd_level"].as<float>();
    Serial.printf("[OWM/weather] grnd_level: %.2f hPa\n", pressaoNova);
  } else if (!doc["main"]["pressure"].isNull()) {
    pressaoNova = doc["main"]["pressure"].as<float>();
    Serial.printf("[OWM/weather] pressure (fallback sea_level): %.2f hPa\n", pressaoNova);
  }

  // Validar e registrar no histórico circular
  if (pressaoNova >= PRESSAO_MIN && pressaoNova <= PRESSAO_MAX) {
    historicoPressao[indexPressao % HIST_PRESSAO] = pressaoNova;
    indexPressao++;
    if (totalLeiturasPressao < HIST_PRESSAO) totalLeiturasPressao++;
    dadosMeteo.pressaoAtmosferica = pressaoNova;
    Serial.printf("[OWM/weather] Pressão registrada: %.2f hPa | histórico: %d/6\n",
                  pressaoNova, totalLeiturasPressao);
  } else {
    Serial.printf("[OWM/weather] Pressão fora do range (%.2f hPa) — ignorada\n", pressaoNova);
  }

  // rain.1h: campo OPCIONAL — presente somente quando chove agora
  // Ausente = sem chuva no momento (não é erro)
  if (!doc["rain"].isNull() && !doc["rain"]["1h"].isNull()) {
    dadosMeteo.chuvaAtualOWM = doc["rain"]["1h"].as<float>();
    Serial.printf("[OWM/weather] rain.1h: %.2f mm/h\n", dadosMeteo.chuvaAtualOWM);
  } else {
    dadosMeteo.chuvaAtualOWM = 0.0f;
    Serial.println(F("[OWM/weather] rain.1h ausente — sem chuva agora"));
  }

  Serial.printf("[OWM/weather] Temp=%.2f°C | Umid=%d%% | Vento=%.2fm/s | %s\n",
                dadosMeteo.temperatura, (int)dadosMeteo.umidadeExterna,
                dadosMeteo.velocidadeVento, dadosMeteo.descricaoTempo.c_str());
  return true;
}

// /forecast — previsão de chuva nas próximas 24h (8 blocos de 3h)
bool buscarForecast() {
  Serial.println(F("[OWM/forecast] Buscando previsão 24h..."));

  // cnt=8 → API retorna exatamente 8 blocos de 3h = 24h de previsão
  // Response completo sem cnt tem 40 blocos (5 dias) e ~20KB
  // HTTPS obrigatório — mesmo host/protocolo do cURL real
  String url = String("https://") + OWM_HOST
               + "/data/2.5/forecast?lat=" + OWM_LAT
               + "&lon=" + OWM_LON
               + "&appid=" + OWM_API_KEY
               + "&lang=pt_br&units=metric&cnt=8";

  Serial.printf("[OWM/forecast] GET %s\n", url.c_str());

  wifiClientSecure.setInsecure();
  HTTPClient http;
  http.begin(wifiClientSecure, url);
  http.setTimeout(15000);
  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[OWM/forecast] Erro HTTP %d — mantendo valor anterior\n", code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  // 10240 bytes: cnt=8 gera ~4-5KB; margem para variações de payload
  DynamicJsonDocument doc(10240);
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    Serial.println(F("[OWM/forecast] Erro ao parsear JSON"));
    return false;
  }

  JsonArray list = doc["list"];
  if (list.isNull() || list.size() == 0) {
    previsao.chuvaFutura24h = 0.0f;
    Serial.println(F("[OWM/forecast] Lista vazia"));
    return false;
  }

  float soma24h = 0.0f;
  int blocos = 0;

  for (JsonObject item : list) {
    float chuvaBloco = 0.0f;

    // rain.3h: OPCIONAL — ausente = 0mm previsto (não é erro)
    if (!item["rain"].isNull() && !item["rain"]["3h"].isNull()) {
      chuvaBloco = item["rain"]["3h"].as<float>();
    } else {
      // Fallback: probabilidade × 3mm (estimativa conservadora)
      float pop = item["pop"].isNull() ? 0.0f : item["pop"].as<float>();
      chuvaBloco = pop * 3.0f;
    }

    soma24h += chuvaBloco;
    blocos++;

    Serial.printf("[OWM/forecast] Bloco %d (%s): %.2fmm (pop=%.0f%%)\n",
                  blocos,
                  item["dt_txt"] | "?",
                  chuvaBloco,
                  (item["pop"].isNull() ? 0.0f : item["pop"].as<float>()) * 100.0f);
  }

  previsao.chuvaFutura24h = soma24h;
  previsao.intensidadePrevisao = obterIntensidadePrevisao();
  previsao.fatorIntensidade = calcularFatorPrevisaoIntensidade(previsao.intensidadePrevisao);
  previsao.timestamp = millis();

  Serial.printf("[OWM/forecast] 24h=%.2fmm | Intensidade: %s | Fator: %.2f\n",
                soma24h, previsao.intensidadePrevisao.c_str(), previsao.fatorIntensidade);
  return true;
}

// ============================================================
//  WIFI E CONECTIVIDADE
// ============================================================
void conectarSistemas() {
  Serial.println(F("🌐 Conectando ao WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(1000);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    Serial.println("\n✓ WiFi conectado — IP: " + WiFi.localIP().toString());
    if (verificarStatusAPI()) Serial.println(F("✓ API própria conectada"));
    else Serial.println(F("⚠️ API própria não responde"));
  } else {
    wifiConectado = false;
    Serial.println(F("\n❌ Falha na conexão WiFi"));
    return;
  }

  wifiClientSecure.setInsecure();
}

void garantirWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[WiFi] Reconectando..."));
    WiFi.reconnect();
    delay(3000);
    wifiConectado = (WiFi.status() == WL_CONNECTED);
  }
}

void verificarConectividade() {
  wifiConectado = (WiFi.status() == WL_CONNECTED);
  if (!wifiConectado) {
    Serial.println(F("⚠️ WiFi desconectado — tentando reconectar..."));
    WiFi.reconnect();
    delay(5000);
    wifiConectado = (WiFi.status() == WL_CONNECTED);
    if (wifiConectado) tentativasReconexao = 0;
    else tentativasReconexao++;
  }

  if (wifiConectado && !apiConectada) verificarStatusAPI();

  static unsigned long ultimoStatus = 0;
  if (millis() - ultimoStatus > 60000UL) {
    mostrarStatusConectividade();
    ultimoStatus = millis();
  }
}

bool verificarStatusAPI() {
  if (!wifiConectado) return false;

  HTTPClient http;
  http.begin(wifiClient, String(API_BASE_URL) + String(API_STATUS_ENDPOINT));
  http.setTimeout(5000);
  int code = http.GET();

  if (code == HTTP_CODE_OK) {
    String resp = http.getString();
    DynamicJsonDocument doc(512);
    if (!deserializeJson(doc, resp) && doc["success"].as<bool>()) {
      apiConectada = true;
      http.end();
      return true;
    }
  }
  apiConectada = false;
  Serial.printf("❌ API não responde: HTTP %d\n", code);
  http.end();
  return false;
}

// ============================================================
//  ENVIO PARA API PRÓPRIA
// ============================================================
bool enviarDadosParaAPI() {
  if (!wifiConectado) {
    Serial.println(F("❌ WiFi desconectado — não é possível enviar dados"));
    return false;
  }

  Serial.println(F("📤 Enviando dados para API..."));
  String payload = criarPayloadJSON();

  HTTPClient http;
  String url = String(API_BASE_URL) + String(API_ENDPOINT);
  http.begin(wifiClient, url);
  http.addHeader("Content-Type", "application/json");
  if (strlen(API_TOKEN) > 0)
    http.addHeader("Authorization", "Bearer " + String(API_TOKEN));
  http.setTimeout(15000);

  Serial.println("🌐 URL: " + url);

  int code = http.POST(payload);

  if (code == HTTP_CODE_OK || code == HTTP_CODE_CREATED) {
    String resp = http.getString();
    Serial.println("✅ Dados enviados com sucesso");
    Serial.println("📄 Resposta: " + resp);
    DynamicJsonDocument doc(512);
    if (!deserializeJson(doc, resp) && doc["success"].as<bool>()) {
      apiConectada = true;
      tentativasEnvioAPI = 0;
      http.end();
      return true;
    }
  } else {
    Serial.printf("❌ Erro ao enviar — HTTP %d\n", code);
    if (code > 0) Serial.println("📄 " + http.getString());
    apiConectada = false;
    tentativasEnvioAPI++;
  }

  http.end();
  return false;
}

// JSON completo com todos os dados
String criarPayloadJSON() {
  DynamicJsonDocument doc(3072);  // FIX v12: aumentado de 2048 para acomodar confiabilidade_detalhes

  doc["timestamp"] = obterTimestampISO();

  // Sensor local
  doc["umidadeSolo"] = dadosLocais.umidadeSolo;
  doc["valorAdc"] = dadosLocais.valorADC;
  doc["sensorOk"] = dadosLocais.sensorOK;
  doc["fatorLocal"] = dadosLocais.fatorLocal;

  // BNDMET
  doc["estacao"] = String(BNDMET_ESTACAO);
  doc["precipitacaoAtual"] = dadosBNDMET.precipitacaoAtual;  // I175
  doc["precipitacao24h"] = dadosBNDMET.precipitacao24h;
  doc["precipitacao7d"] = dadosBNDMET.precipitacao7d;
  doc["precipitacao30d"] = dadosBNDMET.precipitacao30d;
  doc["statusApiBndmet"] = dadosBNDMET.statusAPI;
  doc["qualidadeDadosBndmet"] = dadosBNDMET.qualidadeDados;

  // OpenWeatherMap
  doc["statusApiOwm"] = dadosMeteo.timestamp > 0 ? "OK" : "FALHA";
  doc["temperatura"] = dadosMeteo.temperatura;
  doc["umidadeExterna"] = dadosMeteo.umidadeExterna;
  doc["pressaoAtmosferica"] = roundf(dadosMeteo.pressaoAtmosferica * 10.0f) / 10.0f;
  doc["velocidadeVento"] = dadosMeteo.velocidadeVento;
  doc["chuvaAtualOWM"] = dadosMeteo.chuvaAtualOWM;  // rain.1h
  doc["descricaoTempo"] = dadosMeteo.descricaoTempo;

  // Previsão
  doc["chuvaFutura24h"] = roundf(previsao.chuvaFutura24h * 10.0f) / 10.0f;
  doc["intensidadePrevisao"] = previsao.intensidadePrevisao;
  doc["fatorIntensidade"] = previsao.fatorIntensidade;
  doc["taxaVariacaoUmidade"] = roundf(calcularTaxaVariacao() * 1000.0f) / 1000.0f;

  // Análise de risco
  doc["riscoIntegrado"] = roundf(analiseRisco.riscoIntegrado * 1000.0f) / 1000.0f;
  doc["indiceRisco"] = analiseRisco.indiceRisco;
  doc["nivelAlerta"] = analiseRisco.nivelAlerta;
  doc["amplificado"] = analiseRisco.amplificado;
  doc["recomendacao"] = analiseRisco.recomendacao;
  doc["confiabilidade"] = analiseRisco.confiabilidade;

  // Componentes individuais (para debug e auditoria)
  doc["vLencol"] = roundf(analiseRisco.vLencol * 1000.0f) / 1000.0f;
  doc["vChuvaAtual"] = roundf(analiseRisco.vChuvaAtual * 1000.0f) / 1000.0f;
  doc["vChuvaHistorica"] = roundf(analiseRisco.vChuvaHistorica * 1000.0f) / 1000.0f;
  doc["vChuvaMensal"] = roundf(analiseRisco.vChuvaMensal * 1000.0f) / 1000.0f;
  doc["vChuvaFutura"] = roundf(analiseRisco.vChuvaFutura * 1000.0f) / 1000.0f;
  doc["vTaxaVariacao"] = roundf(analiseRisco.vTaxaVariacao * 1000.0f) / 1000.0f;
  doc["vPressao"] = roundf(analiseRisco.vPressao * 1000.0f) / 1000.0f;

  // Status do sistema
  doc["statusSistema"] = statusSistema;
  doc["buzzerAtivo"] = buzzerAtivo;
  doc["modoManual"] = modoManual;
  doc["wifiConectado"] = wifiConectado;

  // Dados brutos de diagnóstico
  JsonObject brutos = doc.createNestedObject("dadosBrutos");
  brutos["uptime"] = millis();
  brutos["freeHeap"] = ESP.getFreeHeap();
  brutos["rssi"] = WiFi.RSSI();
  brutos["tentativasEnvio"] = tentativasEnvioAPI;
  // FIX v13 (Fix 3): sinaliza janela de cooldown de ruptura.
  // true = sistema ainda em statusTexto=RUPTURA mas umidade já abaixo do limiar,
  // aguardando 3 leituras consecutivas seguras para recalcular o risco normalmente.
  brutos["aguardando_reset_ruptura"] = aguardandoResetRuptura;
  // FIX v13 (Fix 5): distingue "BNDMET não consultado ainda" de "qualidade=0% por falha".
  // false nos registros iniciais (antes da sincronização NTP) indica dados ausentes,
  // não falha da estação. O frontend/banco pode filtrar esses registros.
  brutos["bndmet_inicializado"] = bndmetInicializado;

  // FIX v12: detalhamento do cálculo de confiabilidade — registra cada desconto aplicado.
  // Permite ao operador entender exatamente por que a confiabilidade é X% naquele ciclo.
  // Acessível via: dados_brutos->'confiabilidade_detalhes' no PostgreSQL,
  // ou dadosBrutos.confiabilidade_detalhes no JSON da API.
  JsonObject confDet = brutos.createNestedObject("confiabilidade_detalhes");
  confDet["base"] = 100;
  if (detalhesConfiab.estadoEspecialRuptura) {
    confDet["estado_especial"] = "RUPTURA";
    confDet["total_desconto"] = 0;
    confDet["resultado"] = 100;
  } else {
    confDet["estado_especial"] = (const char*)nullptr;  // null no JSON
    JsonObject desc = confDet.createNestedObject("descontos");
    desc["sensor_falha"] = detalhesConfiab.descontoSensor;
    desc["bndmet_indisponivel"] = detalhesConfiab.descontoBndmetFora;
    desc["qualidade_bndmet"] = detalhesConfiab.descontoQualidadeBndmet;
    desc["owm_indisponivel"] = detalhesConfiab.descontoOWM;
    desc["wifi_desconectado"] = detalhesConfiab.descontoWifi;
    desc["buffer_insuficiente"] = detalhesConfiab.descontoBuffer;
    confDet["total_desconto"] = detalhesConfiab.totalDesconto;
    confDet["resultado"] = detalhesConfiab.resultado;
  }

  String out;
  serializeJson(doc, out);
  return out;
}

String obterTimestampISO() {
  time_t now = time(nullptr);
  if (now < 100000UL) return "";
  struct tm* ti = gmtime(&now);
  char buf[25];
  sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
          ti->tm_hour, ti->tm_min, ti->tm_sec);
  return String(buf);
}

// ============================================================
//  CONFIABILIDADE, RECOMENDAÇÃO E HISTÓRICO
// ============================================================
// ============================================================
//  CONFIABILIDADE DA ANÁLISE — v12
//
//  Definição: grau de completude e atualidade dos dados que
//  alimentaram o cálculo de risco neste ciclo (0–100%).
//
//  Tabela de descontos (pesos somam 100 nas primeiras 5 leituras):
//    -40  Sensor físico com falha       → dado primário ausente
//    -25  BNDMET indisponível           → 3 componentes sem dado real
//    -15  OWM indisponível              → V_ch.futura e V_pressao sem dado
//    -10  WiFi desconectado             → transmissão perdida (cálculo local ocorre)
//    -10  Buffer histórico insuficiente → totalLeiturasSensor < 5
//    MUTUAMENTE EXCLUSIVO com BNDMET indisponível:
//    -10  Qualidade BNDMET < 80%        → dados presentes mas com lacunas
//         (D6594 sempre 73% → desconto fixo no sistema atual)
//
//  Estado especial: RUPTURA → 100% fixo (máxima certeza — hardware confirmou)
//
//  Resultado e detalhes ficam disponíveis em analiseRisco.confiabilidade
//  e detalhesConfiab (serializado em dadosBrutos no JSON)
// ============================================================

void calcularConfiabilidadeAnalise() {
  // ── Estado especial: RUPTURA ──────────────────────────────────────────────
  // Durante ruptura, o sensor físico confirma o estado diretamente.
  // Confiabilidade = 100% independente de qualquer outra falha.
  if (analiseRisco.statusTexto == "RUPTURA") {
    detalhesConfiab = { 0, 0, 0, 0, 0, 0, 0, 100, true };
    analiseRisco.confiabilidade = 100;
    Serial.println(F("  [Conf] RUPTURA — confiabilidade forçada para 100%"));
    return;
  }

  // ── Cálculo normal ────────────────────────────────────────────────────────
  detalhesConfiab.estadoEspecialRuptura = false;

  // Sensor físico
  detalhesConfiab.descontoSensor = dadosLocais.sensorOK ? 0 : 40;

  // BNDMET — indisponível vs qualidade degradada (mutuamente exclusivos)
  if (!dadosBNDMET.apiDisponivel) {
    detalhesConfiab.descontoBndmetFora = 25;
    detalhesConfiab.descontoQualidadeBndmet = 0;  // não avalia qualidade se API caiu
  } else {
    detalhesConfiab.descontoBndmetFora = 0;
    // FIX v12: D6594 qualidade sempre 73% → desconto de -10 é fixo neste sistema
    detalhesConfiab.descontoQualidadeBndmet = (dadosBNDMET.qualidadeDados < 80) ? 10 : 0;
  }

  // OWM — indisponível quando timestamp = 0 (nunca consultado ou falhou)
  bool owmDisponivel = (dadosMeteo.timestamp > 0);
  detalhesConfiab.descontoOWM = owmDisponivel ? 0 : 15;

  // WiFi
  detalhesConfiab.descontoWifi = wifiConectado ? 0 : 10;

  // Buffer histórico — FIX v12: usa contador total não-circular (totalLeiturasSensor)
  // evita o bug da v11 onde indiceHistorico < 5 retornava true ciclicamente a cada 10 análises
  detalhesConfiab.descontoBuffer = (totalLeiturasSensor < 5) ? 10 : 0;

  // ── Soma e resultado ──────────────────────────────────────────────────────
  detalhesConfiab.totalDesconto =
    detalhesConfiab.descontoSensor
    + detalhesConfiab.descontoBndmetFora
    + detalhesConfiab.descontoQualidadeBndmet
    + detalhesConfiab.descontoOWM
    + detalhesConfiab.descontoWifi
    + detalhesConfiab.descontoBuffer;

  detalhesConfiab.resultado = max(0, 100 - detalhesConfiab.totalDesconto);
  analiseRisco.confiabilidade = detalhesConfiab.resultado;

  // ── Log Serial ────────────────────────────────────────────────────────────
  Serial.printf("  [Conf] Base=100");
  if (detalhesConfiab.descontoSensor) Serial.printf(" | -40 sensor");
  if (detalhesConfiab.descontoBndmetFora) Serial.printf(" | -25 BNDMET fora");
  if (detalhesConfiab.descontoQualidadeBndmet) Serial.printf(" | -10 qualidade BNDMET(%d%%)", dadosBNDMET.qualidadeDados);
  if (detalhesConfiab.descontoOWM) Serial.printf(" | -15 OWM fora");
  if (detalhesConfiab.descontoWifi) Serial.printf(" | -10 WiFi");
  if (detalhesConfiab.descontoBuffer) Serial.printf(" | -10 buffer(%d leit.)", totalLeiturasSensor);
  Serial.printf(" = %d%%\n", detalhesConfiab.resultado);
}

void gerarRecomendacaoDetalhada() {
  // Opção C v19: durante simulação, não sobrescreve — preserva texto "[Simulação]"
  if (simulacaoAtiva) return;

  String r;
  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA) {
    r = "🚨 RUPTURA — EVACUAÇÃO IMEDIATA! Umidade acima da linha crítica";
  } else if (analiseRisco.riscoIntegrado > LIMIAR_AMARELO) {
    r = "🔴 CRÍTICO — Evacuar área de risco imediatamente";
  } else if (analiseRisco.riscoIntegrado > LIMIAR_VERDE) {
    if (previsao.chuvaFutura24h > 5.0f)
      r = "🟡 ATENÇÃO — Chuva prevista, aumentar frequência de monitoramento";
    else
      r = "🟡 ATENÇÃO — Situação controlada, manter vigilância";
  } else {
    r = "🟢 NORMAL — Continuar monitoramento de rotina";
  }
  if (dadosBNDMET.precipitacao7d > 50.0f) r += " | Solo saturado por chuvas recentes";
  if (!dadosBNDMET.apiDisponivel) r += " | ⚠️ Dados BNDMET indisponíveis";
  if (analiseRisco.amplificado) r += " | ⚠️ Amplificação de risco ativa";
  analiseRisco.recomendacao = r;
}

void atualizarHistoricoAnalise() {
  historicoRisco[indiceHistorico] = analiseRisco.riscoIntegrado;
  indiceHistorico = (indiceHistorico + 1) % 10;
  Serial.printf("📈 Histórico atualizado — índice: %d\n", indiceHistorico);
}

// ============================================================
//  CALIBRAÇÃO DO SENSOR — EEPROM
// ============================================================
void recalibrarSensor() {
  Serial.println(F("\n🔧 ========== CALIBRAÇÃO DO SENSOR =========="));
  Serial.println(F("S = Solo seco | U = Solo úmido | F = Finalizar | C = Cancelar"));

  int novoSeco = SENSOR_SECO, novoUmido = SENSOR_UMIDO;

  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'S' || cmd == 's') {
        novoSeco = analogRead(PIN_HIGROMETRO);
        Serial.printf("✓ Valor SECO: %d\n", novoSeco);
      } else if (cmd == 'U' || cmd == 'u') {
        novoUmido = analogRead(PIN_HIGROMETRO);
        Serial.printf("✓ Valor ÚMIDO: %d\n", novoUmido);
      } else if (cmd == 'F' || cmd == 'f') {
        if (novoSeco != novoUmido && abs(novoSeco - novoUmido) > 50) {
          SENSOR_SECO = novoSeco;
          SENSOR_UMIDO = novoUmido;
          EEPROM.put(0, SENSOR_SECO);
          EEPROM.put(4, SENSOR_UMIDO);
          EEPROM.commit();
          Serial.printf("✅ Calibração salva — Seco: %d | Úmido: %d | Diff: %d\n",
                        SENSOR_SECO, SENSOR_UMIDO, abs(SENSOR_SECO - SENSOR_UMIDO));
        } else {
          Serial.println(F("⚠️ Calibração inválida — valores não alterados"));
        }
        break;
      } else if (cmd == 'C' || cmd == 'c') {
        Serial.println(F("❌ Calibração cancelada"));
        break;
      }
    }
    delay(100);
  }
  Serial.println(F("🔧 Retornando ao modo normal...\n"));
}

// ============================================================
//  INICIALIZAÇÃO DE MEMÓRIA
// ============================================================
void inicializarEstruturasMemoria() {
  memset(&dadosLocais, 0, sizeof(dadosLocais));
  memset(&dadosBNDMET, 0, sizeof(dadosBNDMET));
  memset(&dadosMeteo, 0, sizeof(dadosMeteo));
  memset(&previsao, 0, sizeof(previsao));
  memset(&analiseRisco, 0, sizeof(analiseRisco));
  Serial.println(F("✓ Estruturas de memória inicializadas"));
}

void inicializarHistorico() {
  indiceHistorico = 0;
  for (int i = 0; i < 10; i++) {
    historicoUmidade[i] = 0;
    historicoPrecipitacao[i] = 0;
    historicoRisco[i] = 0;
  }
  Serial.println(F("✓ Histórico inicializado"));
}

// ============================================================
//  UTILITÁRIOS
// ============================================================
String formatarTempo(unsigned long ts) {
  if (ts == 0) return "Nunca";
  unsigned long s = ts / 1000, m = s / 60, h = m / 60, d = h / 24;
  if (d > 0) return String(d) + "d " + String(h % 24) + "h " + String(m % 60) + "m";
  if (h > 0) return String(h) + "h " + String(m % 60) + "m " + String(s % 60) + "s";
  if (m > 0) return String(m) + "m " + String(s % 60) + "s";
  return String(s) + "s";
}

// ============================================================
//  DISPLAY SERIAL
// ============================================================
void mostrarStatusConectividade() {
  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║         STATUS DO SISTEMA            ║"));
  Serial.println(F("╠══════════════════════════════════════╣"));
  bool owmDisponivel = (dadosMeteo.timestamp > 0);
  Serial.println("║ WiFi   : " + String(wifiConectado ? "✅ CONECTADO" : "❌ DESCONECTADO") + "         ║");
  Serial.println("║ BNDMET : " + String(dadosBNDMET.apiDisponivel ? "✅ OK" : "❌ FALHA") + "                  ║");
  Serial.println("║ OWM    : " + String(owmDisponivel ? "✅ OK" : "❌ Sem dados") + "               ║");
  Serial.println("║ Sensor : " + String(dadosLocais.sensorOK ? "✅ OK" : "⚠️ FALHA") + "                 ║");
  Serial.println("║ API    : " + String(apiConectada ? "✅ Conectada" : "❌ Desconectada") + "      ║");
  Serial.printf("║ Heap   : %d bytes                ║\n", ESP.getFreeHeap());
  Serial.println("║ Uptime : " + formatarTempo(millis()) + "                    ║");
  Serial.println("║ RSSI   : " + String(WiFi.RSSI()) + " dBm                     ║");
  Serial.println(F("╚══════════════════════════════════════╝\n"));
}

void mostrarComandosDisponiveis() {
  Serial.println(F("\n📋 ========== COMANDOS DISPONÍVEIS =========="));
  Serial.println(F("  status   — Status do sistema"));
  Serial.println(F("  analise  — Executar análise de risco"));
  Serial.println(F("  debug    — Análise detalhada (Equações TCC)"));
  Serial.println(F("  calibrar — Calibrar sensor de umidade (EEPROM)"));
  Serial.println(F("  reset    — Resetar sistema"));
  Serial.println(F("  teste    — Teste completo de todos os módulos"));
  Serial.println(F("  enviar         — Forçar envio para API"));
  Serial.println(F("  api            — Testar conexão com API"));
  Serial.println(F("  help           — Este menu"));
  Serial.println(F("── Simulação de nível (demonstração) ────────"));
  Serial.println(F("  alerta verde    — Força nível VERDE  (envio: 60s)"));
  Serial.println(F("  alerta amarelo  — Força nível AMARELO (envio: 20s)"));
  Serial.println(F("  alerta vermelho — Força nível VERMELHO (envio: 10s)"));
  Serial.println(F("  (próxima leitura real do sensor restaura o nível)"));
  Serial.println(F("=============================================\n"));
}

void mostrarAnaliseDetalhada() {
  Serial.println(F("\n╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║       ANÁLISE COMPLETA DE RISCO — TCC            ║"));
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ DADOS LOCAIS DO SENSOR                           ║"));
  Serial.printf("║  Umidade: %.2f%% | ADC: %d | Fator: %.3f          ║\n",
                dadosLocais.umidadeSolo, dadosLocais.valorADC, dadosLocais.fatorLocal);
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ BNDMET — DADOS DE PRECIPITAÇÃO                   ║"));
  Serial.printf("║  Estação: %s | Status: %s              ║\n",
                BNDMET_ESTACAO, dadosBNDMET.statusAPI.c_str());
  Serial.printf("║  Precip. 24h: %.2fmm | 7d: %.2fmm | 30d: %.2fmm  ║\n",
                dadosBNDMET.precipitacao24h, dadosBNDMET.precipitacao7d,
                dadosBNDMET.precipitacao30d);
  Serial.printf("║  Atual (I175): %.2fmm | Qualidade: %d%%           ║\n",
                dadosBNDMET.precipitacaoAtual, dadosBNDMET.qualidadeDados);
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ OPENWEATHERMAP                                   ║"));
  Serial.printf("║  Pressão: %.2f hPa | Temp: %.2f°C              ║\n",
                dadosMeteo.pressaoAtmosferica, dadosMeteo.temperatura);
  Serial.printf("║  Chuva atual (rain.1h): %.2f mm/h               ║\n",
                dadosMeteo.chuvaAtualOWM);
  Serial.printf("║  Forecast 24h: %.2fmm | Intensidade: %-15s          ║\n",
                previsao.chuvaFutura24h, previsao.intensidadePrevisao.c_str());
  Serial.printf("║  Fator intensidade: %.2f                                  ║\n",
                previsao.fatorIntensidade);
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ EQUAÇÃO 5 TCC — COMPONENTES                      ║"));
  Serial.printf("║  V_lençol       = %.3f (%.3f × 0,40)           ║\n",
                analiseRisco.vLencol, dadosLocais.fatorLocal);
  Serial.printf("║  V_ch.atual     = %.3f (%.2fmm × 0,08)         ║\n",
                analiseRisco.vChuvaAtual, dadosBNDMET.precipitacao24h);
  Serial.printf("║  V_ch.histórica = %.3f (%.2fmm × 0,12)         ║\n",
                analiseRisco.vChuvaHistorica, dadosBNDMET.precipitacao7d);
  Serial.printf("║  V_ch.mensal    = %.3f (%.2fmm × 0,10)         ║\n",
                analiseRisco.vChuvaMensal, dadosBNDMET.precipitacao30d);
  Serial.printf("║  V_ch.futura    = %.3f (%.2fmm → %s → %.2f × 0,15)  ║\n",
                analiseRisco.vChuvaFutura, previsao.chuvaFutura24h,
                previsao.intensidadePrevisao.c_str(), previsao.fatorIntensidade);
  Serial.printf("║  V_taxa_var     = %.3f                           ║\n",
                analiseRisco.vTaxaVariacao);
  Serial.printf("║  V_pressao      = %.3f (queda %.2fhPa × 0,05)  ║\n",
                analiseRisco.vPressao, calcularQuedaPressao());
  Serial.printf("║  Amplificado    = %s                           ║\n",
                analiseRisco.amplificado ? "SIM (1,2×)" : "NÃO       ");
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.printf("║  FATOR RISCO = %.3f | ÍNDICE = %d%%             ║\n",
                analiseRisco.riscoIntegrado, analiseRisco.indiceRisco);
  Serial.printf("║  STATUS: %-10s | Confiab: %d%%              ║\n",
                analiseRisco.statusTexto.c_str(), analiseRisco.confiabilidade);
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ LIMITES DE REFERÊNCIA                            ║"));
  Serial.printf("║  Umid. crítica: %.0f%% | Ruptura: %.0f%%         ║\n",
                UMIDADE_CRITICA, UMIDADE_RUPTURA);
  Serial.printf("║  24h: %.0fmm | 7d: %.0fmm | 30d: %.0fmm        ║\n",
                LIMIAR_24H, LIMIAR_7D, LIMIAR_30D);
  Serial.printf("║  Verde: <%.2f | Amarelo: <%.2f | Vermelho: ≥%.2f ║\n",
                LIMIAR_VERDE, LIMIAR_AMARELO, LIMIAR_AMARELO);
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println("║ 💡 " + analiseRisco.recomendacao.substring(0, 44) + " ║");
  Serial.println(F("╚══════════════════════════════════════════════════╝\n"));
}

void testarCalculosTCC() {
  // Teste 6 usa os dados reais já capturados pelos Testes 3, 4 e 5
  // (sensor, BNDMET e OWM rodaram antes neste mesmo comando 'teste')
  Serial.println(F("🧮 Testando cálculos com dados reais capturados..."));

  float u = dadosLocais.umidadeSolo;
  float p24 = dadosBNDMET.precipitacao24h;
  float p7 = dadosBNDMET.precipitacao7d;
  float p30 = dadosBNDMET.precipitacao30d;
  float fc = previsao.chuvaFutura24h;

  float vL = calcularVLencolFreatico(u);
  float vCA = calcularVChuvaAtual(p24);
  float vCH = calcularVChuvaHistorica(p7);
  float vCM = calcularVChuvaMensal(p30);

  // V_ch.futura: calcula inline com fc isolado, sem depender da variável global
  float fintTeste = (fc >= 80.0f)   ? 1.00f
                    : (fc >= 50.0f) ? 0.75f
                    : (fc >= 25.0f) ? 0.50f
                    : (fc >= 5.0f)  ? 0.25f
                                    : 0.00f;
  float vCF = fintTeste * PESO_CH_FUTURA;

  float total = vL + vCA + vCH + vCM + vCF;

  // Amplificação: mesmo critério do modelo real
  float fatorLen = calcularFatorLencolFreatico(u);
  bool amplif = (fatorLen >= LIMIAR_SOLO_SAT) && (fc >= 5.0f);
  if (amplif) total *= FATOR_AMPLIF;

  const char* nivel = (total <= LIMIAR_VERDE)     ? "VERDE"
                      : (total <= LIMIAR_AMARELO) ? "AMARELO"
                                                  : "VERMELHO";

  Serial.printf("  Umidade=%.2f%% P24=%.2fmm P7=%.2fmm P30=%.2fmm FC=%.2fmm\n",
                u, p24, p7, p30, fc);
  Serial.printf("  V_lençol=%.3f V_ch.atual=%.3f V_ch.hist=%.3f\n", vL, vCA, vCH);
  Serial.printf("  V_ch.mensal=%.3f V_ch.futura=%.3f (fator=%.2f) TOTAL=%.3f\n",
                vCM, vCF, fintTeste, total);
  if (amplif)
    Serial.println(F("  ⚠️ Amplificação 1,2× aplicada (solo saturado + previsão ≥ Moderada)"));
  Serial.printf("  ✅ Resultado: LED %s\n", nivel);
}

void debugConectividade() {
  Serial.println(F("\n🔍 ========== DEBUG CONECTIVIDADE =========="));
  Serial.println("Status WiFi: " + String(WiFi.status()));
  if (wifiConectado) {
    Serial.println("  SSID: " + WiFi.SSID());
    Serial.println("  IP: " + WiFi.localIP().toString());
    Serial.println("  RSSI: " + String(WiFi.RSSI()) + " dBm");
  }
  WiFiClient tc;
  Serial.print(F("  Internet (8.8.8.8:53): "));
  Serial.println(tc.connect("8.8.8.8", 53) ? "✅ OK" : "❌ FALHA");
  tc.stop();
  Serial.print(F("  OWM (pro.openweathermap.org:80): "));
  WiFiClient tc2;
  Serial.println(tc2.connect("pro.openweathermap.org", 80) ? "✅ OK" : "❌ FALHA");
  tc2.stop();
  Serial.println(F("=============================================\n"));
}

void executarTesteCompleto() {
  Serial.println(F("\n🧪 ========== TESTE COMPLETO DO SISTEMA =========="));
  Serial.println(F("🔧 Teste 1: Hardware"));
  // Sequência de LEDs
  for (int p : { (int)PIN_LED_VERDE, (int)PIN_LED_AMARELO, (int)PIN_LED_VERMELHO }) {
    digitalWrite(p, HIGH);
    delay(400);
    digitalWrite(p, LOW);
  }
  digitalWrite(PIN_BUZZER, HIGH);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);
  Serial.println(F("  ✅ LEDs e buzzer OK"));

  Serial.println(F("🌐 Teste 2: Conectividade"));
  Serial.println("  WiFi: " + String(wifiConectado ? "✅ OK" : "❌ FALHA"));
  Serial.println("  BNDMET: " + String(dadosBNDMET.apiDisponivel ? "✅ OK" : "❌ FALHA"));
  debugConectividade();

  Serial.println(F("💧 Teste 3: Sensor de umidade"));
  atualizarDadosLocais();
  Serial.printf("  Leitura: %.2f%% | %s\n",
                dadosLocais.umidadeSolo,
                dadosLocais.sensorOK ? "✅ OK" : "⚠️ Verificar calibração");

  Serial.println(F("🌧️ Teste 4: APIs BNDMET"));
  if (wifiConectado) {
    atualizarDadosPrecipitacao();
    Serial.println(dadosBNDMET.apiDisponivel ? "  ✅ BNDMET OK" : "  ❌ BNDMET FALHA");
  } else Serial.println(F("  ⚠️ WiFi desconectado"));

  Serial.println(F("☁️ Teste 5: OpenWeatherMap"));
  if (wifiConectado) {
    atualizarDadosMeteorologicos();
    Serial.printf("  Pressão: %.2f hPa | Forecast 24h: %.2fmm\n",
                  dadosMeteo.pressaoAtmosferica, previsao.chuvaFutura24h);
  } else Serial.println(F("  ⚠️ WiFi desconectado"));

  Serial.println(F("🧮 Teste 6: Cálculos TCC"));
  testarCalculosTCC();

  Serial.println(F("🔍 Teste 7: Análise de risco"));
  analisarRiscoIntegrado();

  Serial.println(F("\n🏁 TESTE COMPLETO FINALIZADO\n"));
}

void resetarSistema() {
  Serial.println(F("\n🔄 Resetando sistema..."));
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AMARELO, LOW);
  digitalWrite(PIN_LED_VERMELHO, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  statusSistema = 0;
  buzzerAtivo = false;
  modoManual = false;
  sistemaInicializado = false;
  inicializarHistorico();
  inicializarEstruturasMemoria();
  Serial.println(F("✓ Reiniciando em 3s..."));
  delay(3000);
  ESP.restart();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║  MONITORAMENTO DE BARRAGEM DE REJEITOS   ║"));
  Serial.println(F("║  ESP8266 NodeMCU | TCC UERJ-IPRJ         ║"));
  Serial.println(F("║  Thamires Ramos dos Santos                ║"));
  Serial.println(F("╚══════════════════════════════════════════╝\n"));

  // EEPROM — calibração do sensor
  EEPROM.begin(512);
  int secoSalvo, umidoSalvo;
  EEPROM.get(0, secoSalvo);
  EEPROM.get(4, umidoSalvo);
  if (secoSalvo > 0 && secoSalvo < 1024 && umidoSalvo > 0 && umidoSalvo < 1024) {
    SENSOR_SECO = secoSalvo;
    SENSOR_UMIDO = umidoSalvo;
    Serial.printf("✓ Calibração da EEPROM — Seco: %d | Úmido: %d\n",
                  SENSOR_SECO, SENSOR_UMIDO);
  } else {
    Serial.printf("✓ Calibração padrão — Seco: %d | Úmido: %d\n",
                  SENSOR_SECO, SENSOR_UMIDO);
  }

  // Hardware
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMARELO, OUTPUT);
  pinMode(PIN_LED_VERMELHO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_HIGROMETRO, INPUT);

  // Teste inicial de componentes (sequencial)
  Serial.println(F("🔧 Testando componentes..."));
  for (int p : { (int)PIN_LED_VERDE, (int)PIN_LED_AMARELO, (int)PIN_LED_VERMELHO }) {
    digitalWrite(p, HIGH);
    delay(400);
    digitalWrite(p, LOW);
  }
  Serial.println(F("✓ LEDs testados"));

  // Conectar WiFi e verificar API
  conectarSistemas();

  // NTP (UTC-3 Brasília)
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print(F("✓ NTP sincronizando"));
  time_t agora = 0;
  int ntpTentativas = 0;
  while (agora < 1000000000UL && ntpTentativas < 20) {
    delay(500);
    Serial.print(".");
    agora = time(nullptr);
    ntpTentativas++;
  }
  Serial.println(agora > 1000000000UL ? " OK" : " Timeout (continuando)");

  // Inicializar estruturas
  inicializarEstruturasMemoria();
  inicializarHistorico();

  // Primeira coleta completa
  Serial.println(F("\nRealizando primeira coleta de dados..."));
  atualizarDadosLocais();

  // Verificar ruptura logo no boot
  if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA) {
    acionarRuptura();
  } else {
    delay(1000);
    atualizarDadosPrecipitacao();
    delay(1000);
    atualizarDadosMeteorologicos();
    analisarRiscoIntegrado();
  }

  // Envio inicial
  if (wifiConectado) {
    Serial.println(F("📤 Enviando dados iniciais..."));
    enviarDadosParaAPI();
  }

  // Inicializar timestamps
  unsigned long agr = millis();
  ultimaLeituraSensor = agr;
  ultimaLeituraBNDMET = agr;
  ultimaLeituraMeteo = agr;
  ultimaOWM = agr;
  ultimaAnalise = agr;
  ultimoEnvioAPI = agr;

  sistemaInicializado = true;
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║         SISTEMA TOTALMENTE INICIADO      ║"));
  Serial.println(F("║   Digite 'help' para ver os comandos     ║"));
  Serial.println(F("╚══════════════════════════════════════════╝\n"));
  mostrarStatusConectividade();
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {
  unsigned long agora = millis();
  static uint8_t contagemRetornoRuptura = 0;

  // Leitura do sensor
  if (agora - ultimaLeituraSensor >= obterIntervaloSensor()) {
    atualizarDadosLocais();
    if (dadosLocais.umidadeSolo >= UMIDADE_RUPTURA) {
      contagemRetornoRuptura = 0;
      aguardandoResetRuptura = false;  // FIX v13: ainda em ruptura ativa, não em cooldown
      acionarRuptura();
    } else if (analiseRisco.nivelAlerta == "VERMELHO" && analiseRisco.statusTexto == "RUPTURA") {
      // Reset de ruptura quando umidade retorna abaixo do limiar.
      // 3 leituras consecutivas seguras → recalcula risco normalmente.
      contagemRetornoRuptura++;
      aguardandoResetRuptura = true;  // FIX v13: sinaliza janela de cooldown para o JSON

      // FIX v14 (DIV-1) — CORRIGIDO v16: mensagem diferenciada durante cooldown.
      // A string anterior usava "RUPTURA — EVACUAÇÃO IMEDIATA" e ">=" mesmo com
      // umidade já abaixo do limiar, gerando inconsistência semântica no banco.
      analiseRisco.recomendacao = "🟡 RETORNO DE RUPTURA — Aguardando confirmação ("
                                  + String(contagemRetornoRuptura) + "/3 leituras seguras"
                                  + " | Umidade atual: " + String(dadosLocais.umidadeSolo, 1)
                                  + "% < " + String(UMIDADE_RUPTURA, 0) + "%)";

      // FIX v14 (DIV-3): recalcula v_taxa durante cooldown.
      // Na v13, v_taxa ficava congelado com o valor do último ciclo de ruptura ativa.
      analiseRisco.vTaxaVariacao = fabsf(calcularTaxaVariacao()) * PESO_TAXA_VAR;

      Serial.printf("🟡 Retorno de ruptura — leitura %d/3 (umidade=%.2f%% < %.2f%%)\n",
                    contagemRetornoRuptura, dadosLocais.umidadeSolo, UMIDADE_RUPTURA);
      if (contagemRetornoRuptura >= 3) {
        contagemRetornoRuptura = 0;
        aguardandoResetRuptura = false;  // FIX v13: cooldown encerrado
        Serial.println(F("🟡 RUPTURA ENCERRADA — Umidade abaixo do limiar por 3 leituras consecutivas"));
        Serial.printf("   Umidade atual: %.2f%% < Limiar: %.2f%%\n",
                      dadosLocais.umidadeSolo, UMIDADE_RUPTURA);
        analisarRiscoIntegrado();
      }
    } else {
      contagemRetornoRuptura = 0;
      aguardandoResetRuptura = false;
      // v19 fix: guard idêntico ao Ponto A — não recalcula durante simulação
      if (!simulacaoAtiva) {
        analisarRiscoIntegrado();
      }
    }
    ultimaLeituraSensor = agora;
  }

  // Dados BNDMET
  // v15: se a última consulta falhou (apiDisponivel=false), usa intervalo
  // reduzido de retry (30s) em vez do intervalo normal adaptativo.
  // Assim o sistema recupera dados pluviométricos mais rapidamente após falha.
  {
    unsigned long intervaloBndmet = dadosBNDMET.apiDisponivel
                                      ? obterIntervaloBNDMET()
                                      : INTERVALO_RETRY_API_FALHA;
    if (agora - ultimaLeituraBNDMET >= intervaloBndmet) {
      atualizarDadosPrecipitacao();
      ultimaLeituraBNDMET = agora;
    }
  }

  // OpenWeatherMap (/weather + /forecast)
  // v15: se OWM nunca foi consultado com sucesso (timestamp=0) ou a última
  // tentativa falhou (timestamp não avançou dentro do intervalo esperado),
  // usa retry acelerado de 30s. owmDisponivel baseia-se no timestamp real
  // de quando os dados foram populados com sucesso, não na temperatura.
  {
    bool owmDisponivel = (dadosMeteo.timestamp > 0);
    unsigned long intervaloOwm = owmDisponivel
                                   ? INTERVALO_OWM
                                   : INTERVALO_RETRY_API_FALHA;
    if (agora - ultimaOWM >= intervaloOwm) {
      atualizarDadosMeteorologicos();
      ultimaOWM = agora;
    }
  }

  // Análise de risco
  if (agora - ultimaAnalise >= INTERVALO_ANALISE) {
    // Opção C v19: enquanto simulacaoAtiva, não recalcula — preserva valores da simulação
    if (!simulacaoAtiva && dadosLocais.umidadeSolo < UMIDADE_RUPTURA) {
      analisarRiscoIntegrado();
    }
    ultimaAnalise = agora;
  }

  // Envio para API
  if (agora - ultimoEnvioAPI >= obterIntervaloEnvioAPI()) {
    if (wifiConectado) {
      bool ok = enviarDadosParaAPI();
      if (!ok && tentativasEnvioAPI > 5) {
        Serial.println(F("🔄 Muitas falhas — verificando conectividade..."));
        verificarConectividade();
      }
      // Opção C v19: decrementa contador de envios da simulação
      if (simulacaoAtiva && ok) {
        simulacaoEnviosRestantes--;
        Serial.printf("🔁 Simulação: %d envio(s) restante(s)\n", simulacaoEnviosRestantes);
        if (simulacaoEnviosRestantes <= 0) {
          simulacaoAtiva = false;
          modoManual = false;
          buzzerAtivo = false;
          noTone(PIN_BUZZER);
          Serial.println(F("🔁 Simulação encerrada — retornando ao modo automático"));
        }
      }
    }
    ultimoEnvioAPI = agora;
  }

  // Buzzer intermitente (quando ativo)
  ativarAlarmeBuzzer();

  // Comandos via Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "status") mostrarStatusConectividade();
    else if (cmd == "analise") analisarRiscoIntegrado();
    else if (cmd == "debug") mostrarAnaliseDetalhada();
    else if (cmd == "calibrar") recalibrarSensor();
    else if (cmd == "reset") resetarSistema();
    else if (cmd == "help") mostrarComandosDisponiveis();
    else if (cmd == "teste") executarTesteCompleto();
    else if (cmd == "enviar") {
      Serial.println(F("🔄 Forçando envio..."));
      enviarDadosParaAPI() ? Serial.println(F("✅ OK")) : Serial.println(F("❌ Falha"));
    } else if (cmd == "api") {
      verificarStatusAPI() ? Serial.println(F("✅ API OK")) : Serial.println(F("❌ API falha"));
    }
    // ── Comandos de simulação de nível (útil para testes e demonstração) ──
    else if (cmd == "alerta verde") {
      // Simulação do nível VERDE com Opção C:
      // simulacaoAtiva bloqueia recálculo e sobrescrita; 3 envios coerentes garantidos.
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 0;

      dadosLocais.umidadeSolo = 5.0f;
      dadosLocais.fatorLocal = 5.0f / UMIDADE_CRITICA;  // 0.200

      analiseRisco.nivelAlerta = "VERDE";
      analiseRisco.statusTexto = "SEGURO";
      analiseRisco.cor = "VERDE";
      analiseRisco.riscoIntegrado = 0.300f;
      analiseRisco.indiceRisco = 30;
      analiseRisco.amplificado = false;
      analiseRisco.vLencol = 0.080f;
      analiseRisco.vChuvaAtual = 0.000f;
      analiseRisco.vChuvaHistorica = dadosBNDMET.precipitacao7d / 150.0f * 0.12f;
      analiseRisco.vChuvaMensal = dadosBNDMET.precipitacao30d / 300.0f * 0.10f;
      analiseRisco.vChuvaFutura = 0.000f;
      analiseRisco.vTaxaVariacao = 0.000f;
      analiseRisco.vPressao = 0.000f;
      analiseRisco.confiabilidade = 100;
      analiseRisco.recomendacao = "🟢 NORMAL — Continuar monitoramento de rotina [Simulação]";

      buzzerAtivo = false;
      noTone(PIN_BUZZER);
      digitalWrite(PIN_LED_VERDE, HIGH);
      digitalWrite(PIN_LED_AMARELO, LOW);
      digitalWrite(PIN_LED_VERMELHO, LOW);

      Serial.println(F("🟢 Simulação: nível forçado para VERDE | modoManual=true | 3 envios"));
      Serial.printf("   Umidade: %.2f%% | Risco: %d%% | Enviando à API...\n",
                    dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      if (enviarDadosParaAPI()) {
        simulacaoEnviosRestantes--;
        Serial.printf("🔁 Simulação: %d envio(s) restante(s)\n", simulacaoEnviosRestantes);
      }

    } else if (cmd == "alerta amarelo") {
      // Simulação do nível AMARELO com Opção C.
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 1;

      dadosLocais.umidadeSolo = 21.0f;
      dadosLocais.fatorLocal = 21.0f / UMIDADE_CRITICA;  // 0.840

      analiseRisco.nivelAlerta = "AMARELO";
      analiseRisco.statusTexto = "ATENÇÃO";
      analiseRisco.cor = "AMARELO";
      analiseRisco.riscoIntegrado = 0.600f;
      analiseRisco.indiceRisco = 60;
      analiseRisco.amplificado = true;
      analiseRisco.vLencol = 0.336f;
      analiseRisco.vChuvaAtual = 0.000f;
      analiseRisco.vChuvaHistorica = dadosBNDMET.precipitacao7d / 150.0f * 0.12f;
      analiseRisco.vChuvaMensal = dadosBNDMET.precipitacao30d / 300.0f * 0.10f;
      analiseRisco.vChuvaFutura = 0.038f;
      analiseRisco.vTaxaVariacao = 0.019f;
      analiseRisco.vPressao = 0.000f;
      analiseRisco.confiabilidade = 100;
      analiseRisco.recomendacao = "🟡 ATENÇÃO — Situação controlada, manter vigilância [Simulação]";

      buzzerAtivo = false;
      noTone(PIN_BUZZER);
      digitalWrite(PIN_LED_VERDE, LOW);
      digitalWrite(PIN_LED_AMARELO, HIGH);
      digitalWrite(PIN_LED_VERMELHO, LOW);

      Serial.println(F("🟡 Simulação: nível forçado para AMARELO | modoManual=true | 3 envios"));
      Serial.printf("   Umidade: %.2f%% | Risco: %d%% | Amplificação: ×1,20 | Enviando à API...\n",
                    dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      if (enviarDadosParaAPI()) {
        simulacaoEnviosRestantes--;
        Serial.printf("🔁 Simulação: %d envio(s) restante(s)\n", simulacaoEnviosRestantes);
      }

    } else if (cmd == "alerta vermelho") {
      // Simulação do nível VERMELHO com Opção C.
      // umidade=27% para não acionar ruptura de hardware (limiar=30%).
      modoManual = true;
      simulacaoAtiva = true;
      simulacaoEnviosRestantes = 3;
      statusSistema = 2;

      dadosLocais.umidadeSolo = 27.0f;
      dadosLocais.fatorLocal = 1.0f;

      analiseRisco.nivelAlerta = "VERMELHO";
      analiseRisco.statusTexto = "CRÍTICO";
      analiseRisco.cor = "VERMELHO";
      analiseRisco.riscoIntegrado = 0.850f;
      analiseRisco.indiceRisco = 85;
      analiseRisco.amplificado = true;
      analiseRisco.vLencol = 0.400f;
      analiseRisco.vChuvaAtual = 0.000f;
      analiseRisco.vChuvaHistorica = dadosBNDMET.precipitacao7d / 150.0f * 0.12f;
      analiseRisco.vChuvaMensal = dadosBNDMET.precipitacao30d / 300.0f * 0.10f;
      analiseRisco.vChuvaFutura = 0.075f;
      analiseRisco.vTaxaVariacao = 0.025f;
      analiseRisco.vPressao = 0.000f;
      analiseRisco.confiabilidade = 100;
      analiseRisco.recomendacao = "🔴 CRÍTICO — Evacuar área de risco imediatamente [Simulação]";

      buzzerAtivo = true;
      digitalWrite(PIN_LED_VERDE, LOW);
      digitalWrite(PIN_LED_AMARELO, LOW);
      digitalWrite(PIN_LED_VERMELHO, HIGH);

      Serial.println(F("🔴 Simulação: nível forçado para VERMELHO | modoManual=true | 3 envios"));
      Serial.printf("   Umidade: %.2f%% | Risco: %d%% | Amplificação: ×1,20 | Enviando à API...\n",
                    dadosLocais.umidadeSolo, analiseRisco.indiceRisco);
      if (enviarDadosParaAPI()) {
        simulacaoEnviosRestantes--;
        Serial.printf("🔁 Simulação: %d envio(s) restante(s)\n", simulacaoEnviosRestantes);
      }
    } else Serial.println(F("❓ Comando não reconhecido. Digite 'help'"));
  }

  // Verificação de conectividade a cada 30s
  static unsigned long ultimaVerif = 0;
  if (agora - ultimaVerif > 30000UL) {
    verificarConectividade();
    ultimaVerif = agora;
  }

  // Watchdog: memória + reconexão
  static unsigned long ultimoWatchdog = 0;
  if (agora - ultimoWatchdog > 300000UL) {
    ultimoWatchdog = agora;
    if (ESP.getFreeHeap() < 5000) {
      Serial.println(F("🚨 MEMÓRIA CRÍTICA — reiniciando..."));
      delay(1000);
      ESP.restart();
    }
    if (!wifiConectado && tentativasReconexao > 10) {
      Serial.println(F("🔄 Reiniciando por falha de conectividade..."));
      ESP.restart();
    }
    if (tentativasEnvioAPI > 10) {
      Serial.println(F("🔄 Muitas falhas de API — reiniciando WiFi..."));
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      tentativasEnvioAPI = 0;
    }
  }

  delay(100);
}

// ============================================================
//  INTERVALOS ADAPTATIVOS POR NÍVEL DE ALERTA
// ============================================================
unsigned long obterIntervaloEnvioAPI() {
  // v15: INTERVALO_ENVIO_TESTE ativo → retorna 10s para todos os níveis.
  // Comentar a linha abaixo para voltar aos intervalos adaptativos de produção.
  // return INTERVALO_ENVIO_TESTE;

  // Intervalos adaptativos de produção (desativados durante testes):
  if (statusSistema >= 2) return INTERVALO_ENVIO_API_VERMELHO;
  if (statusSistema >= 1) return INTERVALO_ENVIO_API_AMARELO;
  return INTERVALO_ENVIO_API_VERDE;
}

unsigned long obterIntervaloSensor() {
  switch (statusSistema) {
    case 2: return INTERVALO_SENSOR_VERMELHO;
    case 1: return INTERVALO_SENSOR_AMARELO;
    default: return INTERVALO_SENSOR_VERDE;
  }
}

unsigned long obterIntervaloBNDMET() {
  switch (statusSistema) {
    case 2: return INTERVALO_BNDMET_VERMELHO;
    case 1: return INTERVALO_BNDMET_AMARELO;
    default: return INTERVALO_BNDMET_VERDE;
  }
}
