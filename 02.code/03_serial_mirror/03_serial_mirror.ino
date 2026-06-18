#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TJpg_Decoder.h>

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

// ====== 串口配置 ======
#define SERIAL_BAUD 2000000

// ====== JPG 最大缓存 ======
#define MAX_JPG_SIZE (120 * 1024)

// ====== 屏幕对象 ======
SPIClass spi = SPIClass(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);

uint8_t *jpgBuf = nullptr;

// 精确读取指定字节数
bool readBytesExact(uint8_t *buf, size_t len, uint32_t timeoutMs = 5000)
{
  size_t received = 0;
  uint32_t lastTime = millis();

  while (received < len) {
    int availableBytes = Serial.available();

    if (availableBytes > 0) {
      size_t need = len - received;
      size_t toRead = availableBytes < need ? availableBytes : need;

      int n = Serial.readBytes(buf + received, toRead);
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

void showWaitingScreen()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.println("Waiting JPG");

  tft.setTextSize(1);
  tft.setCursor(20, 80);
  tft.println("Protocol: JPG0");
  tft.setCursor(20, 100);
  tft.println("Screen: 280x240");
}

// 接收并显示一帧 JPG
void receiveJpg()
{
  uint8_t header[8];

  // 协议：
  // JPG0 + 4字节JPG长度，大端 + JPG数据
  if (!readBytesExact(header, 8, 3000)) {
    Serial.println("ERR: header timeout");
    return;
  }

  if (header[0] != 'J' || header[1] != 'P' || header[2] != 'G' || header[3] != '0') {
    Serial.println("ERR: bad header");

    while (Serial.available()) {
      Serial.read();
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
    return;
  }

  Serial.print("OK JPG SIZE: ");
  Serial.println(jpgLen);

  if (!readBytesExact(jpgBuf, jpgLen, 8000)) {
    Serial.println("ERR: jpg data timeout");
    return;
  }

  Serial.println("RX DONE");

  // 投屏时不要每帧清屏，否则会闪
  JRESULT result = TJpgDec.drawJpg(0, 0, jpgBuf, jpgLen);

  if (result == JDR_OK) {
    Serial.println("DONE");
  } else {
    Serial.print("ERR: jpg decode failed ");
    Serial.println(result);
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1000);

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

  // Adafruit 版本一般用 false
  // 如果颜色异常，改成 true 再试
  TJpgDec.setSwapBytes(false);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(jpgOutput);

  showWaitingScreen();

  Serial.println("READY");
}

void loop()
{
  if (Serial.available() >= 8) {
    receiveJpg();
  }
}