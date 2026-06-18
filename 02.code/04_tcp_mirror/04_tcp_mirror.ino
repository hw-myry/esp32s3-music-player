#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TJpg_Decoder.h>

// ====== WiFi 配置 ======
const char* ssid = "MagentaWLAN-7LSH_2.4";
const char* password = "44573549123221016562";

// ====== TCP 端口，必须和 Qt 一致 ======
#define TCP_PORT 8081

WiFiServer server(TCP_PORT);

// ====== ST7789 接线 ======
#define TFT_SCLK 21
#define TFT_MOSI 14
#define TFT_CS   47
#define TFT_DC   48
#define TFT_RST  13
#define TFT_BL   45

// ====== 屏幕物理尺寸 ======
#define TFT_INIT_W 240
#define TFT_INIT_H 280

// ====== 旋转 90 度后的显示尺寸 ======
#define LCD_W 280
#define LCD_H 240

// ====== 图片显示偏移 ======
#define IMAGE_OFFSET_X 0
#define IMAGE_OFFSET_Y 0

// ====== 串口只用于调试 ======
#define SERIAL_BAUD 115200

// ====== JPG 最大缓存 ======
#define MAX_JPG_SIZE (120 * 1024)

// ====== 屏幕对象 ======
SPIClass spi = SPIClass(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);

uint8_t *jpgBuf = nullptr;

// ====== FPS 统计 ======
uint32_t frameCount = 0;
uint32_t fpsTimer = 0;

// 从 TCP 精确读取指定字节数
bool readBytesExact(WiFiClient &client, uint8_t *buf, size_t len, uint32_t timeoutMs)
{
  size_t received = 0;
  uint32_t lastTime = millis();

  while (received < len) {
    if (!client.connected()) {
      Serial.println("ERR: client disconnected while reading");
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
      Serial.print("ERR: read timeout, received ");
      Serial.print(received);
      Serial.print(" / ");
      Serial.println(len);
      return false;
    }

    delay(0);
  }

  return true;
}

// JPG 解码输出回调
bool jpgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
  int screenW = tft.width();
  int screenH = tft.height();

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

void showBootScreen()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 30);
  tft.println("WiFi TCP JPG");

  tft.setTextSize(1);
  tft.setCursor(20, 70);
  tft.println("Connecting WiFi...");
}

void showWifiInfo()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 30);
  tft.println("TCP Server");

  tft.setTextSize(1);
  tft.setCursor(20, 70);
  tft.print("SSID: ");
  tft.println(WiFi.SSID());

  tft.setCursor(20, 90);
  tft.print("IP: ");
  tft.println(WiFi.localIP());

  tft.setCursor(20, 110);
  tft.print("Port: ");
  tft.println(TCP_PORT);

  tft.setCursor(20, 140);
  tft.println("Waiting client...");
}

void showClientOk()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.println("Client OK");
}

// 接收并显示一帧 JPG
bool receiveJpg(WiFiClient &client)
{
  uint8_t header[8];

  // 协议：
  // JPG0 + 4字节 JPG 长度，大端 + JPG 数据
  if (!readBytesExact(client, header, 8, 5000)) {
    Serial.println("ERR: header timeout");
    client.println("ERR: header timeout");
    return false;
  }

  if (header[0] != 'J' || header[1] != 'P' || header[2] != 'G' || header[3] != '0') {
    Serial.println("ERR: bad header");
    client.println("ERR: bad header");

    while (client.available()) {
      client.read();
    }

    return false;
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

    return false;
  }

  Serial.print("JPG SIZE: ");
  Serial.println(jpgLen);

  Serial.println("Start reading JPG data...");

  if (!readBytesExact(client, jpgBuf, jpgLen, 15000)) {
    Serial.println("ERR: jpg data timeout");
    client.println("ERR: jpg data timeout");
    return false;
  }

  Serial.println("JPG data received.");
  Serial.println("Start decode...");

  uint32_t t0 = millis();

  // 这里不要套 tft.startWrite / tft.endWrite，
  // 避免和 Adafruit_ST7789 的 drawRGBBitmap 产生冲突
  JRESULT result = TJpgDec.drawJpg(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, jpgBuf, jpgLen);

  uint32_t t1 = millis();

  Serial.print("Decode result: ");
  Serial.println(result);

  if (result == JDR_OK) {
    frameCount++;

    uint32_t now = millis();
    if (now - fpsTimer >= 1000) {
      Serial.print("FPS: ");
      Serial.print(frameCount);
      Serial.print(" , decode+draw ms: ");
      Serial.print(t1 - t0);
      Serial.print(" , jpg size: ");
      Serial.println(jpgLen);

      frameCount = 0;
      fpsTimer = now;
    }

    client.print("DONE\n");
    client.flush();
    return true;
  } else {
    Serial.print("ERR: jpg decode failed ");
    Serial.println(result);

    client.print("ERR: jpg decode failed ");
    client.println(result);

    return false;
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Port: ");
  Serial.println(TCP_PORT);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  jpgBuf = (uint8_t *)malloc(MAX_JPG_SIZE);
  if (jpgBuf == nullptr) {
    while (1) {
      Serial.println("ERR: malloc jpgBuf failed");
      delay(1000);
    }
  }

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  spi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  // 物理屏幕初始化 240x280
  tft.init(TFT_INIT_W, TFT_INIT_H, SPI_MODE0);
  tft.setSPISpeed(40000000);

  // 旋转 90 度，显示区域变成 280x240
  tft.setRotation(3);

  // 开机彩屏自测
  tft.fillScreen(ST77XX_RED);
  delay(200);
  tft.fillScreen(ST77XX_GREEN);
  delay(200);
  tft.fillScreen(ST77XX_BLUE);
  delay(200);

  TJpgDec.setSwapBytes(false);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(jpgOutput);

  showBootScreen();

  connectWiFi();

  server.begin();
  server.setNoDelay(true);

  showWifiInfo();

  Serial.println("READY TCP SERVER");
}

void loop()
{
  WiFiClient client = server.available();

  if (client) {
    client.setNoDelay(true);

    Serial.println("Client connected");
    client.println("READY");

    showClientOk();

    while (client.connected()) {
      if (client.available() > 0) {
        bool ok = receiveJpg(client);

        if (!ok) {
          Serial.println("Receive frame failed.");
          delay(10);
        }
      } else {
        delay(1);
      }
    }

    client.stop();

    Serial.println("Client disconnected");

    showWifiInfo();
  }
}