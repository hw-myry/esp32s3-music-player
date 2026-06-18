#include <Arduino.h>
#include <WiFi.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TJpg_Decoder.h>

// =======================
// WiFi 配置
// =======================
const char* ssid = "MagentaWLAN-7LSH_2.4";
const char* password = "44573549123221016562";

// TCP 端口，要和 Qt 一致
#define TCP_PORT 8081
WiFiServer server(TCP_PORT);

// =======================
// ST7789 接线
// =======================
#define TFT_SCLK 21
#define TFT_MOSI 14
#define TFT_CS   47
#define TFT_DC   48
#define TFT_RST  13
#define TFT_BL   45

// 物理屏初始化尺寸
#define LCD_W 240
#define LCD_H 280

// 如果显示方向不对，改 0 / 1 / 2 / 3
// 你的 Qt 端如果发 280x240 横屏图，通常用 1
#define TFT_ROTATION 3

#define MAX_JPG_SIZE (120 * 1024)

SPIClass spi = SPIClass(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);

uint8_t *jpgBuf = nullptr;

// =======================
// 精确读取 TCP 数据
// =======================
bool readBytesExact(WiFiClient &client, uint8_t *buf, size_t len, uint32_t timeoutMs = 5000)
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

// =======================
// JPG 解码输出到屏幕
// =======================
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
  tft.println("TCP JPG");

  tft.setTextSize(1);
  tft.setCursor(20, 80);
  tft.println("Protocol: JPG0");

  tft.setCursor(20, 100);
  tft.print("Port: ");
  tft.println(TCP_PORT);

  tft.setCursor(20, 120);
  tft.print("IP: ");
  tft.println(WiFi.localIP());
}

// =======================
// 接收一张 JPG
// =======================
void receiveJpg(WiFiClient &client)
{
  uint8_t header[8];

  if (!readBytesExact(client, header, 8, 3000)) {
    Serial.println("ERR: header timeout");
    client.println("ERR: header timeout");
    return;
  }

  if (header[0] != 'J' || header[1] != 'P' || header[2] != 'G' || header[3] != '0') {
    Serial.println("ERR: bad header");
    client.println("ERR: bad header");

    while (client.available()) {
      client.read();
    }

    return;
  }

  uint32_t jpgLen =
      ((uint32_t)header[4] << 24) |
      ((uint32_t)header[5] << 16) |
      ((uint32_t)header[6] << 8)  |
      ((uint32_t)header[7]);

  if (jpgLen == 0 || jpgLen > MAX_JPG_SIZE) {
    Serial.print("ERR: bad jpg size ");
    Serial.println(jpgLen);

    client.print("ERR: bad jpg size ");
    client.println(jpgLen);
    return;
  }

  Serial.print("OK JPG SIZE: ");
  Serial.println(jpgLen);

  if (!readBytesExact(client, jpgBuf, jpgLen, 10000)) {
    Serial.println("ERR: jpg data timeout");
    client.println("ERR: jpg data timeout");
    return;
  }

  Serial.println("RX DONE");

  tft.fillScreen(ST77XX_BLACK);

  JRESULT result = TJpgDec.drawJpg(0, 0, jpgBuf, jpgLen);

  if (result == JDR_OK) {
    Serial.println("DONE");
    client.println("DONE");
  } else {
    Serial.print("ERR: jpg decode failed ");
    Serial.println(result);

    client.print("ERR: jpg decode failed ");
    client.println(result);
  }
}

// =======================
// WiFi 连接
// =======================
void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Connecting WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Port: ");
  Serial.println(TCP_PORT);
}

// =======================
// setup
// =======================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 TCP JPG Receiver");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  spi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(LCD_W, LCD_H, SPI_MODE0);
  tft.setSPISpeed(40000000);
  tft.setRotation(TFT_ROTATION);

  tft.fillScreen(ST77XX_RED);
  delay(300);
  tft.fillScreen(ST77XX_GREEN);
  delay(300);
  tft.fillScreen(ST77XX_BLUE);
  delay(300);
  tft.fillScreen(ST77XX_BLACK);

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(false);
  TJpgDec.setCallback(tft_output);

  connectWiFi();

  jpgBuf = (uint8_t *)malloc(MAX_JPG_SIZE);
  if (jpgBuf == nullptr) {
    while (1) {
      Serial.println("ERR: malloc jpgBuf failed");
      delay(1000);
    }
  }

  showWaiting();

  server.begin();
  server.setNoDelay(true);

  Serial.println("READY TCP JPG SERVER");
}

// =======================
// loop
// =======================
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

  while (client.connected()) {
    if (client.available() >= 8) {
      receiveJpg(client);
    } else {
      delay(1);
    }
  }

  client.stop();
  Serial.println("Client disconnected");
}