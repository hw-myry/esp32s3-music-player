#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <TJpg_Decoder.h>

#define TFT_SCLK 21
#define TFT_MOSI 14
#define TFT_CS   47
#define TFT_DC   48
#define TFT_RST  13
#define TFT_BL   45

#define LCD_W 240
#define LCD_H 280

#define SERIAL_BAUD 921600
#define MAX_JPG_SIZE (120 * 1024)

SPIClass spi = SPIClass(FSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&spi, TFT_CS, TFT_DC, TFT_RST);

uint8_t *jpgBuf = nullptr;

bool readBytesExact(uint8_t *buf, size_t len, uint32_t timeoutMs = 5000) {
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

// TJpg_Decoder 解码出来一块图像后，会调用这里
bool jpgOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= LCD_H) {
    return false;
  }

  if (x >= LCD_W) {
    return false;
  }

  // 防止越界
  if (x + w > LCD_W) {
    w = LCD_W - x;
  }

  if (y + h > LCD_H) {
    h = LCD_H - y;
  }

  tft.drawRGBBitmap(x, y, bitmap, w, h);
  return true;
}

void showWaitingScreen() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 40);
  tft.println("Waiting JPG");

  tft.setTextSize(1);
  tft.setCursor(20, 80);
  tft.println("Protocol: JPG0");
}

void receiveJpg() {
  uint8_t header[8];

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

  tft.fillScreen(ST77XX_BLACK);

  JRESULT result = TJpgDec.drawJpg(0, 0, jpgBuf, jpgLen);

  if (result == JDR_OK) {
    Serial.println("DONE");
  } else {
    Serial.print("ERR: jpg decode failed ");
    Serial.println(result);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(1000);

  jpgBuf = (uint8_t *)malloc(MAX_JPG_SIZE);
  if (jpgBuf == nullptr) {
    while (1) {
      Serial.println("ERR: malloc failed");
      delay(1000);
    }
  }

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  spi.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(LCD_W, LCD_H, SPI_MODE0);
  tft.setSPISpeed(40000000);
  tft.setRotation(0);

  // 先自测 Adafruit 是否正常
  tft.fillScreen(ST77XX_RED);
  delay(300);
  tft.fillScreen(ST77XX_GREEN);
  delay(300);
  tft.fillScreen(ST77XX_BLUE);
  delay(300);
  tft.fillScreen(ST77XX_BLACK);

  // Adafruit 先用 false
  // 如果后面颜色异常，再改成 true
  TJpgDec.setSwapBytes(false);
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(jpgOutput);

  showWaitingScreen();

  Serial.println("READY");
}

void loop() {
  if (Serial.available() >= 8) {
    receiveJpg();
  }
}