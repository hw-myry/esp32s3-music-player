#include <Arduino.h>
#include <WiFi.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TJpg_Decoder.h>

#include <driver/i2s.h>
#include <esp_idf_version.h>
#include <esp_heap_caps.h>

// ============================================================
// 网络模式选择
// ============================================================
// 1 = WiFi 模式：ESP32 连接路由器，Qt 连接串口打印出来的 ESP32 IP
// 2 = 热点模式：ESP32 自己开热点，Qt 连接 192.168.4.1
#define NET_MODE 2

#if (NET_MODE != 1) && (NET_MODE != 2)
#error "NET_MODE must be 1 (WiFi STA) or 2 (WiFi AP)"
#endif

// ============================================================
// WiFi / 热点 配置
// ============================================================
// NET_MODE = 1 时使用：连接路由器
const char* staSsid = "MagentaWLAN-7LSH_2.4";
const char* staPassword = "44573549123221016562";

// NET_MODE = 2 时使用：ESP32 自己开热点
const char* apSsid = "ESP32_JPG_PCM";
const char* apPassword = "12345678";   // 至少 8 位

IPAddress apLocalIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

#define AP_CHANNEL 6
#define AP_MAX_CONNECTIONS 1

#define TCP_PORT 8081
WiFiServer server(TCP_PORT);

// ============================================================
// ST7789 接线
// ============================================================
#define TFT_SCLK 21
#define TFT_MOSI 14
#define TFT_CS   47
#define TFT_DC   48
#define TFT_RST  13
#define TFT_BL   45

#define LCD_W 240
#define LCD_H 280

// Qt 发的是 280x240 横屏图片，通常用 1。
// 方向不对就改 0 / 2 / 3。
#define TFT_ROTATION 3

#define MAX_JPG_SIZE (120 * 1024)

// ============================================================
// MAX98357 I2S 接线
// ============================================================
#define I2S_BCLK  17
#define I2S_LRC   18
#define I2S_DOUT  16

#define AUDIO_SAMPLE_RATE 16000
#define MAX_PCM_PACKET_SIZE 4096
#define RING_BUFFER_SIZE 32768
#define START_BUFFER_BYTES 4096
#define AUDIO_BLOCK_SAMPLES 512

SPIClass spi = SPIClass(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);

uint8_t *jpgBuf = nullptr;
uint8_t *pcmPacketBuf = nullptr;
uint8_t *ringBuf = nullptr;

volatile size_t rbHead = 0;
volatile size_t rbTail = 0;
volatile size_t rbCount = 0;

portMUX_TYPE rbMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool clientOnline = false;
volatile bool audioStarted = false;

uint32_t underrunCount = 0;
uint32_t overflowCount = 0;

// ============================================================
// 内存申请：优先 PSRAM，没有就普通 RAM
// ============================================================
uint8_t* allocBuffer(size_t size, const char *name)
{
  uint8_t *p = nullptr;

  p = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!p) {
    p = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_8BIT);
  }

  if (!p) {
    Serial.print("ERR malloc failed: ");
    Serial.print(name);
    Serial.print(" size=");
    Serial.println(size);
    return nullptr;
  }

  Serial.print("malloc OK: ");
  Serial.print(name);
  Serial.print(" size=");
  Serial.println(size);

  return p;
}

// ============================================================
// Ring Buffer
// ============================================================
size_t ringAvailable()
{
  size_t v;
  portENTER_CRITICAL(&rbMux);
  v = rbCount;
  portEXIT_CRITICAL(&rbMux);
  return v;
}

void ringClear()
{
  portENTER_CRITICAL(&rbMux);
  rbHead = 0;
  rbTail = 0;
  rbCount = 0;
  portEXIT_CRITICAL(&rbMux);
}

void ringDropOldestInsideCritical(size_t len)
{
  len &= ~1;

  if (len == 0) {
    return;
  }

  if (len > rbCount) {
    rbHead = 0;
    rbTail = 0;
    rbCount = 0;
    return;
  }

  rbTail = (rbTail + len) % RING_BUFFER_SIZE;
  rbCount -= len;
}

void ringWrite(const uint8_t *data, size_t len)
{
  len &= ~1;

  if (len == 0) {
    return;
  }

  if (len > RING_BUFFER_SIZE) {
    data += len - RING_BUFFER_SIZE;
    len = RING_BUFFER_SIZE;
    len &= ~1;
  }

  portENTER_CRITICAL(&rbMux);

  size_t freeSpace = RING_BUFFER_SIZE - rbCount;

  if (len > freeSpace) {
    size_t dropLen = len - freeSpace;
    dropLen &= ~1;
    ringDropOldestInsideCritical(dropLen);
    overflowCount++;
  }

  size_t firstPart = min(len, RING_BUFFER_SIZE - rbHead);
  memcpy(ringBuf + rbHead, data, firstPart);

  size_t secondPart = len - firstPart;
  if (secondPart > 0) {
    memcpy(ringBuf, data + firstPart, secondPart);
  }

  rbHead = (rbHead + len) % RING_BUFFER_SIZE;
  rbCount += len;

  portEXIT_CRITICAL(&rbMux);
}

size_t ringRead(uint8_t *out, size_t len)
{
  len &= ~1;

  if (len == 0) {
    return 0;
  }

  portENTER_CRITICAL(&rbMux);

  size_t canRead = rbCount;
  if (len > canRead) {
    len = canRead;
    len &= ~1;
  }

  size_t firstPart = min(len, RING_BUFFER_SIZE - rbTail);
  memcpy(out, ringBuf + rbTail, firstPart);

  size_t secondPart = len - firstPart;
  if (secondPart > 0) {
    memcpy(out + firstPart, ringBuf, secondPart);
  }

  rbTail = (rbTail + len) % RING_BUFFER_SIZE;
  rbCount -= len;

  portEXIT_CRITICAL(&rbMux);

  return len;
}

// ============================================================
// TCP 精确读取
// ============================================================
bool readBytesExact(WiFiClient &client, uint8_t *buf, size_t len, uint32_t timeoutMs)
{
  size_t received = 0;
  uint32_t lastTime = millis();

  while (received < len) {
    if (!client.connected()) {
      return false;
    }

    int availableBytes = client.available();

    if (availableBytes > 0) {
      size_t need = len - received;
      size_t toRead = availableBytes < need ? availableBytes : need;

      int n = client.read(buf + received, toRead);

      if (n > 0) {
        received += n;
        lastTime = millis();
      }
    }

    if (millis() - lastTime > timeoutMs) {
      return false;
    }

    delay(0);
  }

  return true;
}

uint32_t readU32BE(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) |
         ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8)  |
         ((uint32_t)p[3]);
}

// ============================================================
// ST7789 / JPG
// ============================================================
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
  int16_t screenW = tft.width();
  int16_t screenH = tft.height();

  if (x >= screenW || y >= screenH) {
    return false;
  }

  if (x + w > screenW) {
    w = screenW - x;
  }

  if (y + h > screenH) {
    h = screenH - y;
  }

  tft.drawRGBBitmap(x, y, bitmap, w, h);
  return true;
}

void showWaiting()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.println("TCP Ready");

  tft.setTextSize(1);
  tft.setCursor(20, 80);
  tft.println("JPG0 image");
  tft.setCursor(20, 100);
  tft.println("PCM0 audio");

  tft.setCursor(20, 130);
#if NET_MODE == 1
  tft.print("WiFi IP: ");
  tft.println(WiFi.localIP());
#else
  tft.print("AP IP: ");
  tft.println(WiFi.softAPIP());
#endif

  tft.setCursor(20, 150);
  tft.print("Port: ");
  tft.println(TCP_PORT);

#if NET_MODE == 2
  tft.setCursor(20, 170);
  tft.print("SSID: ");
  tft.println(apSsid);
#endif
}

bool receiveJpgPacket(WiFiClient &client, uint32_t jpgLen)
{
  if (jpgLen == 0 || jpgLen > MAX_JPG_SIZE) {
    Serial.print("ERR: bad jpg size ");
    Serial.println(jpgLen);

    client.print("ERR: bad jpg size ");
    client.println(jpgLen);
    return false;
  }

  Serial.print("RX JPG SIZE: ");
  Serial.println(jpgLen);

  if (!readBytesExact(client, jpgBuf, jpgLen, 10000)) {
    Serial.println("ERR: jpg data timeout");
    client.println("ERR: jpg data timeout");
    return false;
  }

  Serial.println("RX JPG DONE");

  tft.fillScreen(ST77XX_BLACK);

  JRESULT result = TJpgDec.drawJpg(0, 0, jpgBuf, jpgLen);

  if (result == JDR_OK) {
    Serial.println("DONE");
    client.println("DONE");
    return true;
  }

  Serial.print("ERR: jpg decode failed ");
  Serial.println(result);

  client.print("ERR: jpg decode failed ");
  client.println(result);
  return false;
}

// ============================================================
// I2S / 音频
// ============================================================
void setupI2S()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,

#if ESP_IDF_VERSION_MAJOR >= 4
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
    .communication_format = I2S_COMM_FORMAT_I2S,
#endif

    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err;

  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("i2s_driver_install failed: ");
    Serial.println(err);
    while (1) {
      delay(1000);
    }
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.print("i2s_set_pin failed: ");
    Serial.println(err);
    while (1) {
      delay(1000);
    }
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
  Serial.println("I2S ready");
}

void audioTask(void *param)
{
  static int16_t monoBuf[AUDIO_BLOCK_SAMPLES];
  static int16_t stereoBuf[AUDIO_BLOCK_SAMPLES * 2];

  const size_t monoBytesNeed = AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
  const size_t stereoBytes = AUDIO_BLOCK_SAMPLES * 2 * sizeof(int16_t);

  while (true) {
    if (!clientOnline) {
      audioStarted = false;
      memset(stereoBuf, 0, stereoBytes);

      size_t written = 0;
      i2s_write(I2S_NUM_0, stereoBuf, stereoBytes, &written, portMAX_DELAY);

      vTaskDelay(1);
      continue;
    }

    size_t available = ringAvailable();

    if (!audioStarted) {
      if (available >= START_BUFFER_BYTES) {
        audioStarted = true;
      } else {
        memset(stereoBuf, 0, stereoBytes);

        size_t written = 0;
        i2s_write(I2S_NUM_0, stereoBuf, stereoBytes, &written, portMAX_DELAY);

        vTaskDelay(1);
        continue;
      }
    }

    size_t got = ringRead((uint8_t *)monoBuf, monoBytesNeed);

    if (got < monoBytesNeed) {
      memset(((uint8_t *)monoBuf) + got, 0, monoBytesNeed - got);
      audioStarted = false;
      underrunCount++;
    }

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      int16_t s = monoBuf[i];
      stereoBuf[i * 2 + 0] = s;
      stereoBuf[i * 2 + 1] = s;
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_NUM_0, stereoBuf, stereoBytes, &bytesWritten, portMAX_DELAY);
  }
}

bool receivePcmPacket(WiFiClient &client, uint32_t pcmLen)
{
  if (pcmLen == 0 || pcmLen > MAX_PCM_PACKET_SIZE || (pcmLen % 2) != 0) {
    Serial.print("ERR: bad PCM size ");
    Serial.println(pcmLen);

    client.print("ERR: bad PCM size ");
    client.println(pcmLen);
    return false;
  }

  if (!readBytesExact(client, pcmPacketBuf, pcmLen, 3000)) {
    Serial.println("ERR: PCM data timeout");
    return false;
  }

  ringWrite(pcmPacketBuf, pcmLen);
  return true;
}

// ============================================================
// TCP 包分发
// ============================================================
bool receiveOnePacket(WiFiClient &client)
{
  uint8_t header[8];

  if (!readBytesExact(client, header, 8, 5000)) {
    Serial.println("ERR: header timeout");
    return false;
  }

  uint32_t payloadLen = readU32BE(header + 4);

  if (header[0] == 'J' && header[1] == 'P' && header[2] == 'G' && header[3] == '0') {
    Serial.print("HEADER JPG0 len=");
    Serial.println(payloadLen);
    return receiveJpgPacket(client, payloadLen);
  }

  if (header[0] == 'P' && header[1] == 'C' && header[2] == 'M' && header[3] == '0') {
    return receivePcmPacket(client, payloadLen);
  }

  Serial.print("ERR: bad header ");
  Serial.write(header, 4);
  Serial.println();

  client.print("ERR: bad header ");
  client.write(header, 4);
  client.println();

  while (client.available()) {
    client.read();
  }

  return false;
}

// ============================================================
// WiFi / 热点
// ============================================================
void startNetwork()
{
  WiFi.setSleep(false);

#if NET_MODE == 1
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting WiFi: ");
  Serial.println(staSsid);

  WiFi.begin(staSsid, staPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi STA connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Port: ");
  Serial.println(TCP_PORT);

#elif NET_MODE == 2
  WiFi.mode(WIFI_AP);

  Serial.print("Starting hotspot: ");
  Serial.println(apSsid);

  if (!WiFi.softAPConfig(apLocalIP, apGateway, apSubnet)) {
    Serial.println("softAPConfig failed");
    while (1) {
      delay(1000);
    }
  }

  bool ok = WiFi.softAP(apSsid, apPassword, AP_CHANNEL, false, AP_MAX_CONNECTIONS);

  if (!ok) {
    Serial.println("softAP start failed");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("WiFi AP started");
  Serial.print("SSID: ");
  Serial.println(apSsid);
  Serial.print("Password: ");
  Serial.println(apPassword);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Port: ");
  Serial.println(TCP_PORT);
#endif
}

// ============================================================
// setup
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 TCP JPG + PCM Receiver");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  spi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(LCD_W, LCD_H, SPI_MODE0);
  tft.setSPISpeed(40000000);
  tft.setRotation(TFT_ROTATION);

  tft.fillScreen(ST77XX_RED);
  delay(200);
  tft.fillScreen(ST77XX_GREEN);
  delay(200);
  tft.fillScreen(ST77XX_BLUE);
  delay(200);
  tft.fillScreen(ST77XX_BLACK);

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);
  TJpgDec.setCallback(tft_output);

  // 先启动网络，再申请大内存，避免 WiFi task 创建失败
  startNetwork();

  Serial.print("Free heap before malloc: ");
  Serial.println(ESP.getFreeHeap());

  jpgBuf = allocBuffer(MAX_JPG_SIZE, "jpgBuf");
  pcmPacketBuf = allocBuffer(MAX_PCM_PACKET_SIZE, "pcmPacketBuf");
  ringBuf = allocBuffer(RING_BUFFER_SIZE, "ringBuf");

  if (!jpgBuf || !pcmPacketBuf || !ringBuf) {
    Serial.println("ERR: buffer malloc failed");
    while (1) {
      delay(1000);
    }
  }

  Serial.print("Free heap after malloc: ");
  Serial.println(ESP.getFreeHeap());

  setupI2S();

  xTaskCreatePinnedToCore(
    audioTask,
    "audioTask",
    4096,
    NULL,
    3,
    NULL,
    0
  );

  showWaiting();

  server.begin();
  server.setNoDelay(true);

  Serial.println("READY TCP JPG + PCM SERVER");
}

// ============================================================
// loop
// ============================================================
void loop()
{
  WiFiClient client = server.available();

  if (!client) {
    delay(1);
    return;
  }

  client.setNoDelay(true);

  Serial.println("Client connected");
  client.println("READY");

  ringClear();
  clientOnline = true;
  audioStarted = false;
  underrunCount = 0;
  overflowCount = 0;

  uint32_t statTimer = millis();

  while (client.connected()) {
    if (client.available() > 0) {
      bool ok = receiveOnePacket(client);

      if (!ok) {
        Serial.println("Receive packet failed, close client");
        break;
      }
    } else {
      delay(1);
    }

    uint32_t now = millis();
    if (now - statTimer >= 1000) {
      Serial.print("Audio Buffer: ");
      Serial.print(ringAvailable());
      Serial.print(" bytes | Underrun: ");
      Serial.print(underrunCount);
      Serial.print(" | Overflow: ");
      Serial.println(overflowCount);

      statTimer = now;
    }
  }

  client.stop();

  clientOnline = false;
  audioStarted = false;
  ringClear();

  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("Client disconnected");
}