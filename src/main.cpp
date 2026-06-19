#include <Arduino.h>
#include <Wire.h> // Biblioteca serial I2C do compilador (permitida, como Serial)
                  // Justificativa: abstrai os registradores I2C do ESP32
                  // (I2C_SCL_LOW_PERIOD_REG, I2C_COMMAND_REG, etc.) de forma
                  // portável. Clock configurado em 400 kHz (Fast mode)

//  Configurações de Hardware
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SSD1306_ADDR 0x3C // Endereço I2C do display (SA0 em GND)
#define DHT11_PIN 4
#define INTERN_LED_PIN 2

//  Variáveis do DHT11
static uint8_t dhtData[5]; // 2 bytes para umidade, 2 para temperatura e 1 para
                           // verificação de erros
static float temperatura = 0.0f;
static float umidade = 0.0f;
static bool dhtErro = true; //  Começa verdadeira para
                            // impedir que a tela exiba dados falsos ou nulos
                            // antes da primeira leitura válida do sensor.

//  Variáveis de temporização
static unsigned long lastDHTRead = 0;
static unsigned long lastDisplayUpd = 0;
const unsigned long DHT_INTERVAL = 2500UL; // leitura a cada 2,5 s
const unsigned long DISP_INTERVAL = 500UL; // tela atualiza a cada 50 ms

//  Relógio
// Prescaler = 80  →  f_timer = 80 MHz / 80 = 1 MHz  →  período = 1 µs
// Alarme    = 1.000.000 ciclos × 1 µs = 1 s exato
static volatile uint8_t seg = 0;
static volatile uint8_t minuto = 0;
static volatile uint8_t hora = 0;
static hw_timer_t *timer0 = NULL;
static portMUX_TYPE timerMux =
    portMUX_INITIALIZER_UNLOCKED; // controle de exclusão mútua

void ARDUINO_ISR_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  seg++;
  if (seg >= 60) {
    seg = 0;
    minuto++;
  }
  if (minuto >= 60) {
    minuto = 0;
    hora++;
  }
  if (hora >= 24)
    hora = 0;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FRAMEBUFFER — toda a tela é desenhada aqui primeiro, depois enviada de uma
//  vez
// ═══════════════════════════════════════════════════════════════════════════════
// O SSD1306 128×64 tem 8 páginas × 128 colunas = 1024 bytes de GDDRAM.
// Cada byte representa 8 pixels verticais dentro de uma página.
// Organização do framebuffer: fb[página * 128 + coluna]
static uint8_t fb[1024];

// Limpa o framebuffer em RAM (sem I2C)
static void fb_clear() { memset(fb, 0, sizeof(fb)); }

// Acende/apaga pixel individual no framebuffer
static void fb_pixel(uint8_t x, uint8_t y, bool on) {
  if (x >= 128 || y >= 64)
    return;
  uint16_t idx = (y / 8) * 128 + x;
  uint8_t bit = 1 << (y % 8);
  if (on)
    fb[idx] |= bit;
  else
    fb[idx] &= ~bit;
}

static void fb_hline(uint8_t x0, uint8_t x1, uint8_t y) {
  for (uint8_t x = x0; x <= x1; x++)
    fb_pixel(x, y, true);
}

// Envia o framebuffer inteiro para o display em uma única sequência I2C
// Wire.write aceita até 32 bytes por transmissão. A solução: usa múltiplas
// transmissões de 32 bytes, cada uma com o byte de controle 0x40, totalizando
// 32 transações de 33 bytes.
static void fb_flush() {
  // Configura janela de endereçamento: todas as colunas e páginas
  Wire.beginTransmission(SSD1306_ADDR);
  Wire.write(0x00); // comandos seguintes servem para configurar a janela
                    // física de memória RAM interna do display.
  Wire.write(
      0x21); // Configurar os limites de colunas da janela de renderização
  Wire.write(0x00);
  Wire.write(0x7F); // colunas 0-127
  Wire.write(0x22); // Configura os limites de páginas verticais da página 0 até
                    // a página 7
  Wire.write(0x00);
  Wire.write(0x07); // páginas 0-7
  Wire.endTransmission();

  // Envia 1024 bytes em blocos de 32
  for (uint16_t i = 0; i < 1024; i += 32) {
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x40);       // o byte vai ser um pixel
    Wire.write(&fb[i], 32); // passando pro display o que está escrito nos bytes
    Wire.endTransmission();
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DRIVER SSD1306 — comandos de inicialização via I2C
// ═══════════════════════════════════════════════════════════════════════════════
// O SSD1306 recebe cada transação I2C assim:
//   START | endereço+W | byte-controle | dado(s) | STOP
// Byte de controle:
//   0x00 → próximo(s) byte(s) são COMANDOS
//   0x40 → próximo(s) byte(s) são DADOS (GDDRAM)

static void oled_cmd(uint8_t c) {
  Wire.beginTransmission(SSD1306_ADDR); // COMEÇA A CONVERSAR COM O DISPLAY
  Wire.write(0x00);
  Wire.write(c);
  Wire.endTransmission();
}

// Sequência de inicialização do SSD1306
// Referência: datasheet SSD1306 Rev 1.1, seção 8.8 "Application Example"
static void oled_init() {
  oled_cmd(0xAE); // Display OFF
  oled_cmd(0xD5);
  oled_cmd(0x80); // Clock: divisor=1, fosc=8
  oled_cmd(0xA8);
  oled_cmd(0x3F); // Mux ratio = 64 linhas (0x3F = 63)
  oled_cmd(0xD3);
  oled_cmd(0x00); // Display offset = 0
  oled_cmd(0x40); // Start line = 0
  oled_cmd(0x8D);
  oled_cmd(0x14); // Charge pump ATIVADO (Vcc interno)
  oled_cmd(0x20);
  oled_cmd(0x00); // Modo de endereçamento: Horizontal
  oled_cmd(0xA1); // Segment remap: col 127 → SEG0
  oled_cmd(0xC8); // COM scan direction: remapped
  oled_cmd(0xDA);
  oled_cmd(0x12); // COM pins: alternativo, sem remap
  oled_cmd(0x81);
  oled_cmd(0xCF); // Contraste = 0xCF
  oled_cmd(0xD9);
  oled_cmd(0xF1); // Pre-charge: fase1=1, fase2=15
  oled_cmd(0xDB);
  oled_cmd(0x40); // Vcomh deselect = 0,89 × Vcc
  oled_cmd(0xA4); // Exibe GDDRAM
  oled_cmd(0xA6); // Modo não é invertido
  oled_cmd(0xAF); // Display ON
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FONTE BITMAP 5×7 — Origem dessa informação: Adafruit-GFX-Library
// ═══════════════════════════════════════════════════════════════════════════════
// Cada caractere: 5 colunas. Cada byte = 8 pixels verticais.
// Armazenada em Flash (PROGMEM) para não consumir RAM.

static const uint8_t font5x7[][5] PROGMEM = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' ' 0x20
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ')'
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
    {0x00, 0x08, 0x14, 0x22, 0x41}, // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // '='
    {0x41, 0x22, 0x14, 0x08, 0x00}, // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x03, 0x04, 0x78, 0x04, 0x03}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x00, 0x00, 0x7F, 0x41, 0x41}, // '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x41, 0x41, 0x7F, 0x00, 0x00}, // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 'f'
    {0x08, 0x14, 0x54, 0x54, 0x3C}, // 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // '}'
    {0x08, 0x08, 0x2A, 0x1C, 0x08}, // '~'
};

//  Desenha caractere 1× no framebuffer
// x: coluna inicial (0-127), y: linha inicial (0-63)
static void fb_putc(uint8_t x, uint8_t y, char c) {
  if (c < 0x20 || c > 0x7E)
    c = '?';
  for (uint8_t col = 0; col < 5; col++) {
    uint8_t glyph = pgm_read_byte(&font5x7[c - 0x20][col]);
    for (uint8_t row = 0; row < 8; row++)
      if (glyph & (1 << row))
        fb_pixel(x + col, y + row, true);
  }
  // coluna de espaço já fica zerada pelo fb_clear()
}

//  Desenha string 1× no framebuffer
static void fb_puts(uint8_t x, uint8_t y, const char *s) {
  while (*s) {
    fb_putc(x, y, *s++);
    x += 6;
  } // 5 px glifo + 1 px espaço
}

//  Desenha caractere 2× no framebuffer (escala por software)
// Cada bit da fonte vira 2×2 pixels no framebuffer.
// x: coluna inicial, y: linha inicial (ocupa 16 linhas verticais)
static void fb_putc2x(uint8_t x, uint8_t y, char c) {
  if (c < 0x20 || c > 0x7E)
    c = '?';
  for (uint8_t col = 0; col < 5; col++) {
    uint8_t glyph = pgm_read_byte(&font5x7[c - 0x20][col]);
    for (uint8_t row = 0; row < 8; row++) {
      if (glyph & (1 << row)) {
        // Cada pixel vira um bloco 2×2
        fb_pixel(x + col * 2, y + row * 2, true);
        fb_pixel(x + col * 2 + 1, y + row * 2, true);
        fb_pixel(x + col * 2, y + row * 2 + 1, true);
        fb_pixel(x + col * 2 + 1, y + row * 2 + 1, true);
      }
    }
  }
}

//  Desenha string 2× no framebuffer
static void fb_puts2x(uint8_t x, uint8_t y, const char *s) {
  while (*s) {
    fb_putc2x(x, y, *s++);
    x += 12;
  } // 5×2 px + 2 px espaço
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Leitura do DHT11 por Registradores
// ═══════════════════════════════════════════════════════════════════════════════
static void lerDHT11() {
  uint32_t bit_times[40] = {};

  // INICIALIZAÇÃO
  GPIO.enable_w1ts =
      (1u << DHT11_PIN); // Quero o bit 4 como saída pra startar o dht
  GPIO.out_w1tc = (1u << DHT11_PIN); // Zera o bit 4 do dht
  delay(20);                         // delay pra acordar o dht
  GPIO.out_w1ts =
      (1u << DHT11_PIN); // depois do delay de 20ms, o dht foi inicializado
  delayMicroseconds(40); // tempo pro dht ver que o tempo foi liberado
  GPIO.enable_w1tc = (1u << DHT11_PIN); // o dht começa a controlar o fio, então
                                        // passo o pino pra entrada

  portDISABLE_INTERRUPTS();

  int64_t t;

  // Checar se o dht está funcioando
  t = esp_timer_get_time();
  while (!((GPIO.in >> DHT11_PIN) & 1)) { // sai em um HIgh - recebeu a mensagem
    if ((esp_timer_get_time() - t) > 500) {
      portENABLE_INTERRUPTS();
      dhtErro = true;
      return;
    }
  }
  t = esp_timer_get_time();
  while ((GPIO.in >> DHT11_PIN) &
         1) { // sai em um Low - ta pronto pra transmitir  a mensage
    if ((esp_timer_get_time() - t) > 500) {
      portENABLE_INTERRUPTS();
      dhtErro = true;
      return;
    }
  }

  // INicializando
  for (int i = 0; i < 40; i++) {
    t = esp_timer_get_time();
    while (!((GPIO.in >> DHT11_PIN) & 1)) {
      if ((esp_timer_get_time() - t) > 500) {
        portENABLE_INTERRUPTS();
        dhtErro = true;
        return;
      }
    }

    // Guarda quanto tempo o bit fic=ou em High, pra saber se é 0 ou 1
    // dependendo do tempo
    int64_t t_high = esp_timer_get_time();
    while ((GPIO.in >> DHT11_PIN) & 1) {
      if ((esp_timer_get_time() - t_high) > 500) {
        portENABLE_INTERRUPTS();
        dhtErro = true;
        return;
      }
    }
    bit_times[i] = (uint32_t)(esp_timer_get_time() - t_high);
  }

  portENABLE_INTERRUPTS();

  // Coloca as durações no DHT
  memset(dhtData, 0, sizeof(dhtData));
  for (int i = 0; i < 40; i++) {
    dhtData[i / 8] <<= 1; // empurra os bits para a esquerda
    if (bit_times[i] > 40)
      dhtData[i / 8] |= 1;
  }

  // Checando se deu certo
  uint8_t checksum = (dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF;
  if (dhtData[4] == checksum && checksum != 0) {
    umidade = (float)dhtData[0];
    temperatura = (float)dhtData[2];
    dhtErro = false;
  } else {
    dhtErro = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Atualização do Display
// ═══════════════════════════════════════════════════════════════════════════════
static void atualizarDisplay() {
  portENTER_CRITICAL(&timerMux);
  uint8_t h = hora, m = minuto, s = seg;
  portEXIT_CRITICAL(&timerMux);

  char buf[20];

  fb_clear(); // apaga

  // Escrevendo o Relógio e informado a localização
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
  fb_puts(40, 0, buf);

  // linha horizontal
  fb_hline(0, 127, 9);

  // Temperatura e Umidade
  if (dhtErro) {
    fb_puts(0, 13, "Erro DHT11");
    fb_puts(0, 23, "Verificar conexao");
  } else {
    snprintf(buf, sizeof(buf), "T:%.1fC", temperatura);
    fb_puts2x(0, 13, buf);

    snprintf(buf, sizeof(buf), "U:%.1f%%", umidade);
    fb_puts2x(0, 30, buf);
  }

  fb_hline(0, 127, 54);

  fb_puts(0, 56, dhtErro ? "Status: ERRO    " : "Status: OK      ");

  // Envia TUDO para o display de uma vez
  fb_flush();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  // Serial: 115200 baud — taxa padrão dos conversores USB-Serial do ESP32
  // DevKit. 8N1: 8 bits de dados, sem paridade, 1 stop bit (padrão universal
  // UART).
  Serial.begin(115200);

  // LED interno via registrador
  GPIO.enable_w1ts = (1u << INTERN_LED_PIN);
  GPIO.out_w1tc = (1u << INTERN_LED_PIN);

  // I2C: SDA=21, SCL=22 (pinos padrão ESP32 DevKit), 400 kHz (Fast mode)
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Aguarda estabilização do Vcc do display após power-on
  delay(100);
  oled_init();

  // Envia framebuffer zerado para limpar qualquer lixo na GDDRAM
  fb_clear();
  fb_flush();

  // Timer0: prescaler=80 → 1 MHz → alarme a cada 1 s
  // Cálculo: 80 MHz / 80 = 1 MHz; 1.000.000 ciclos × 1µs = 1 s
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTimer, true);
  timerAlarmWrite(timer0, 1000000, true);
  timerAlarmEnable(timer0);

  Serial.println(F("Sistema iniciado."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Loop principal
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Leitura do sensor a cada 2,5 s
  if (now - lastDHTRead >= DHT_INTERVAL) {
    lastDHTRead = now;
    GPIO.out_w1ts = (1u << INTERN_LED_PIN);
    lerDHT11();
    GPIO.out_w1tc = (1u << INTERN_LED_PIN);

    // pra evitar problema com a icr
    portENTER_CRITICAL(&timerMux);
    uint8_t h = hora, m = minuto, s = seg;
    portEXIT_CRITICAL(&timerMux);

    if (dhtErro)
      Serial.printf("[%02d:%02d:%02d] ERRO na leitura do DHT11\n", h, m, s);
    else
      Serial.printf("[%02d:%02d:%02d] Temp: %.1fC | Umid: %.1f%%\n", h, m, s,
                    temperatura, umidade);
  }

  // Atualiza display a cada 500 ms
  if (now - lastDisplayUpd >= DISP_INTERVAL) {
    lastDisplayUpd = now;
    atualizarDisplay();
  }
}