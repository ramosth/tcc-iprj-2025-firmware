# Firmware do protótipo físico — TCC UERJ/IPRJ

Firmware do protótipo físico do sistema de alerta para barragens de rejeitos
(ESP8266 NodeMCU V3, Arduino IDE). Versão usada no TCC: `tcc_versao_21`.

Modelo de alerta (limiar hidrometeorológico de duas variáveis):

- Eq. 1: `S = (L - L_seco) / (L_sat - L_seco)` — saturação relativa do solo (higrômetro no pino A0)
- Eq. 2: `P72 = P48,obs + P24,prev` — chuva observada (BNDMET, estação D6594) + prevista (OpenWeatherMap)
- Eq. 3: VERMELHO se `S >= 0,86` e `P72 >= 100 mm`; AMARELO se `S >= 0,64` e `P72 >= 60 mm`; VERDE nos demais casos
- Regras de contingência para falta do higrômetro e/ou dos dados de chuva

Ligações: higrômetro A0; LED azul (nível VERDE) D1; LED amarelo D2; LED vermelho D3; buzzer ativo TMB-12A03 D4.

## Como usar

1. Copie `tcc_versao_21/secrets.h.example` para `tcc_versao_21/secrets.h` e preencha a rede Wi-Fi, o endereço do backend e as chaves das APIs (`secrets.h` não vai para o Git).
2. Abra `tcc_versao_21/tcc_versao_21.ino` na Arduino IDE, placa "NodeMCU 1.0 (ESP-12E Module)", 115200 baud.
3. `FATOR_DEMO = 1` opera nos tempos reais (60 min / 10 min). Nos testes do TCC foi usado `FATOR_DEMO = 120` (30 s / 5 s).

Comandos no monitor serial: `status`, `calibrar [seco|sat|reset]`, `chuva <mm>`, `chuva parcial <mm>`, `chuva off`, `chuva na`, `help`.

A versão anterior (`tcc_versao_19.ino`) continua na branch `main` apenas como histórico.
