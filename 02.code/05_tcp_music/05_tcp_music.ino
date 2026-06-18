#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>

// ====== WiFi 配置 ======
const char* ssid = "MagentaWLAN-7LSH_2.4";
const char* password = "44573549123221016562";

// ====== TCP 端口，必须和 Qt 一致 ======
#define TCP_PORT 8081
WiFiServer server(TCP_PORT);

// ====== MAX98357 I2S 接线 ======
#define I2S_BCLK  17
#define I2S_LRC   18
#define I2S_DOUT  16

// ====== 音频参数，必须和 Qt 发送端一致 ======
#define AUDIO_SAMPLE_RATE 16000

// Qt 发送的 PCM0 一包最大长度
#define MAX_PCM_PACKET_SIZE 8192

// ====== 环形缓冲区 ======
// 64KB 大概能缓存 2 秒左右 16k/16bit/mono 音频
// 如果编译后内存不够，改成 32768
#define RING_BUFFER_SIZE 65536

// 播放前至少缓存多少字节，防止 WiFi 抖动
#define START_BUFFER_BYTES 8192

// I2S 每次写多少个单声道采样点
#define AUDIO_BLOCK_SAMPLES 512

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

// ====== Ring Buffer ======

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

void ringDropOldest(size_t len)
{
  len &= ~1;

  if (len == 0) return;

  if (len > rbCount) {
    rbHead = 0;
    rbTail = 0;
    rbCount = 0;
    return;
  }

  rbTail = (rbTail + len) % RING_BUFFER_SIZE;
  rbCount -= len;
}

// 写入环形缓冲区，不够空间时丢弃最旧数据
void ringWrite(const uint8_t *data, size_t len)
{
  len &= ~1;

  if (len == 0) return;

  if (len > RING_BUFFER_SIZE) {
    data += (len - RING_BUFFER_SIZE);
    len = RING_BUFFER_SIZE;
    len &= ~1;
  }

  portENTER_CRITICAL(&rbMux);

  size_t freeSpace = RING_BUFFER_SIZE - rbCount;

  if (len > freeSpace) {
    size_t dropLen = len - freeSpace;
    dropLen &= ~1;

    ringDropOldest(dropLen);
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

// 从环形缓冲区读取
size_t ringRead(uint8_t *out, size_t len)
{
  len &= ~1;

  if (len == 0) return 0;

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

// ====== TCP 精确读取 ======

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

// ====== I2S ======

void setupI2S()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,

    // 用立体声输出更稳，后面会把 mono 复制成 L/R
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,

#if ESP_IDF_VERSION_MAJOR >= 4
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
    .communication_format = I2S_COMM_FORMAT_I2S,
#endif

    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,

    // 加大 DMA，减少卡顿
    .dma_buf_count = 16,
    .dma_buf_len = 1024,

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
    while (1) delay(1000);
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.print("i2s_set_pin failed: ");
    Serial.println(err);
    while (1) delay(1000);
  }

  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("I2S ready");
}

// ====== 音频播放任务 ======

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

    // mono 复制成 stereo：L = R = sample
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      int16_t s = monoBuf[i];
      stereoBuf[i * 2 + 0] = s;
      stereoBuf[i * 2 + 1] = s;
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_NUM_0, stereoBuf, stereoBytes, &bytesWritten, portMAX_DELAY);
  }
}

// ====== WiFi ======

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

// ====== 接收 PCM0 包 ======

bool receiveOnePcmPacket(WiFiClient &client)
{
  uint8_t header[8];

  if (!readBytesExact(client, header, 8, 3000)) {
    return false;
  }

  if (header[0] != 'P' || header[1] != 'C' || header[2] != 'M' || header[3] != '0') {
    Serial.print("Bad header: ");
    Serial.write(header, 4);
    Serial.println();

    while (client.available()) {
      client.read();
    }

    return false;
  }

  uint32_t pcmLen =
      ((uint32_t)header[4] << 24) |
      ((uint32_t)header[5] << 16) |
      ((uint32_t)header[6] << 8)  |
      ((uint32_t)header[7]);

  if (pcmLen == 0 || pcmLen > MAX_PCM_PACKET_SIZE || (pcmLen % 2) != 0) {
    Serial.print("Bad PCM size: ");
    Serial.println(pcmLen);
    return false;
  }

  if (!readBytesExact(client, pcmPacketBuf, pcmLen, 3000)) {
    return false;
  }

  ringWrite(pcmPacketBuf, pcmLen);

  return true;
}

// ====== setup / loop ======

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 WiFi PCM Audio Player - Ring Buffer Version");

  pcmPacketBuf = (uint8_t *)malloc(MAX_PCM_PACKET_SIZE);
  ringBuf = (uint8_t *)malloc(RING_BUFFER_SIZE);

  if (!pcmPacketBuf) {
    Serial.println("malloc pcmPacketBuf failed");
    while (1) delay(1000);
  }

  if (!ringBuf) {
    Serial.println("malloc ringBuf failed");
    Serial.println("Try changing RING_BUFFER_SIZE from 65536 to 32768");
    while (1) delay(1000);
  }

  setupI2S();
  connectWiFi();

  xTaskCreatePinnedToCore(
    audioTask,
    "audioTask",
    4096,
    NULL,
    3,
    NULL,
    0
  );

  server.begin();
  server.setNoDelay(true);

  Serial.println("READY TCP PCM SERVER");
}

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

  uint32_t packetCount = 0;
  uint32_t statTimer = millis();

  while (client.connected()) {
    if (client.available() > 0) {
      bool ok = receiveOnePcmPacket(client);

      if (ok) {
        packetCount++;
      } else {
        Serial.println("Receive packet failed");
        delay(5);
      }
    } else {
      delay(0);
    }

    uint32_t now = millis();
    if (now - statTimer >= 1000) {
      Serial.print("Packets/s: ");
      Serial.print(packetCount);
      Serial.print(" | Buffer: ");
      Serial.print(ringAvailable());
      Serial.print(" bytes | Underrun: ");
      Serial.print(underrunCount);
      Serial.print(" | Overflow: ");
      Serial.println(overflowCount);

      packetCount = 0;
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