#include <Arduino.h>

#include <SD.h>

#include <FS.h>

#include "driver/i2s.h"

// === Константы ===

// === Настройки I2S ===
#define I2S_SAMPLE_RATE 22050 // Частота дискретизации
#define I2S_SAMPLE_BITS 16 // Битность выборки
#define I2S_CHANNEL_NUM 1
#define I2S_BUFFER_LEN 1024 // Размер буфера I2S

// === Пины подключения радиостанции ===
#define pinPPT 32
#define pinSQL 12

// === Пины подключения SD-карты и I2S ===
#define SD_CS_PIN 5
#define I2S_DOUT_PIN 25
#define I2S_BCLK_PIN 27
#define I2S_LRCK_PIN 26

const char * audioFileName = "/recording.wav"; // Имя файла для записи
File audioFile;

bool Rec = false; // Флаг записи

// === Конфигурация встроенного I2S (пин 36, ADC) ===
void setupI2S_Input() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_BUFFER_LEN / 2,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_NUM_0, & config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
  i2s_adc_enable(I2S_NUM_0);
}

// === Конфигурация PCM5102A (DAC) ===
void setupI2S_Output() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_BUFFER_LEN / 2,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_NUM_0, & config, 0, NULL);
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCK_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(I2S_NUM_0, & pin_config);
}

// === Прописывание wav заголовка ===
void writeWavHeader(File file, uint32_t dataSize) {
  uint32_t sampleRate = I2S_SAMPLE_RATE;
  uint16_t bitsPerSample = I2S_SAMPLE_BITS;
  uint16_t channels = I2S_CHANNEL_NUM;
  uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
  uint16_t blockAlign = channels * (bitsPerSample / 8);
  uint32_t chunkSize = 36 + dataSize;

  file.seek(0);
  file.write((const uint8_t * )
    "RIFF", 4);
  file.write((uint8_t * ) & chunkSize, 4);
  file.write((const uint8_t * )
    "WAVE", 4);
  file.write((const uint8_t * )
    "fmt ", 4);

  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1;

  file.write((uint8_t * ) & subchunk1Size, 4);
  file.write((uint8_t * ) & audioFormat, 2);
  file.write((uint8_t * ) & channels, 2);
  file.write((uint8_t * ) & sampleRate, 4);
  file.write((uint8_t * ) & byteRate, 4);
  file.write((uint8_t * ) & blockAlign, 2);
  file.write((uint8_t * ) & bitsPerSample, 2);
  file.write((const uint8_t * )
    "data", 4);
  file.write((uint8_t * ) & dataSize, 4);

}

// === Функция записи на карту ===
void recordWav() {
  uint8_t i2sBuffer[I2S_BUFFER_LEN];
  int16_t samples[I2S_BUFFER_LEN / 2];
  size_t bytesRead;
  uint32_t totalBytes = 0;

  float scaleFactor = 16.0 f;
  float fc = 70.0 f;
  float fs = I2S_SAMPLE_RATE;
  float RC = 1.0 f / (2.0 f * PI * fc);
  float alpha = RC / (RC + 1.0 f / fs);

  int16_t x_prev = 0, y_prev = 0;

  Serial.println("Начинаем запись...");

  unsigned long lastSignalTime = millis(); // Время последнего сигнала
  const unsigned long signalTimeout = 1000; // Таймаут "тишины" в миллисекундах

  while (true) {
    bool signal = digitalRead(pinSQL);
    if (signal) {
      lastSignalTime = millis(); // Обновляем время последнего сигнала
    }

    if (millis() - lastSignalTime > signalTimeout) {
      // Если сигнал не поступает в течение заданного времени (1000 мс), завершаем запись
      break;
    }

    if (i2s_read(I2S_NUM_0, i2sBuffer, I2S_BUFFER_LEN, & bytesRead, portMAX_DELAY) != ESP_OK)
      continue;

    int sampleCount = bytesRead / 2;
    for (int i = 0; i < sampleCount; ++i) {
      uint16_t raw = ((uint16_t) i2sBuffer[i * 2 + 1] << 8) | i2sBuffer[i * 2];
      int16_t centered = (int16_t) raw - 2048;
      int32_t scaled = centered * scaleFactor;
      scaled = constrain(scaled, -32768, 32767);
      int16_t current = scaled;
      int16_t filtered = alpha * (y_prev + current - x_prev);
      x_prev = current;
      y_prev = filtered;
      samples[i] = filtered;
    }

    audioFile.write((uint8_t * ) samples, sampleCount * sizeof(int16_t));
    totalBytes += sampleCount * sizeof(int16_t);
  }

  Serial.println("Запись завершена.");
  writeWavHeader(audioFile, totalBytes);
}

// === Функция воспроизведения ===
void playWav(const char * filePath) {
  File file = SD.open(filePath, FILE_READ);
  if (!file || file.size() <= 44) {
    Serial.println("Ошибка WAV-файла.");
    return;
  }

  uint8_t buffer[I2S_BUFFER_LEN];
  size_t bytesRead, bytesWritten;

  Serial.print("Воспроизведение: ");
  Serial.println(filePath);

  file.seek(44); // Пропускаем заголовок WAV, так как и так знаем параметры записи, он нужен только для упрощения воспроизведения на компьютере
  while (file.available()) {
    bytesRead = file.read(buffer, I2S_BUFFER_LEN);
    i2s_write(I2S_NUM_0, buffer, bytesRead, & bytesWritten, portMAX_DELAY);
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
  file.close();
  Serial.println("Готово.");
}

void setup() {
  Serial.begin(115200);

  pinMode(pinPPT, INPUT);
  pinMode(pinSQL, INPUT);

  Serial.print("SD карта: "); // Инициализация SD-карты
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ошибка инициализации... проверьте подключение.");
    while (true);
  }
  Serial.println("репитер готов к исползованию!");

}

void loop() {
  bool SQL = digitalRead(pinSQL);

  //Условие для начала записи
  if (SQL && !Rec) {
    Rec = true;

    if (SD.exists(audioFileName)) SD.remove(audioFileName);
    audioFile = SD.open(audioFileName, FILE_WRITE);
    if (!audioFile) {
      Serial.println("Не удалось открыть файл.");
      return;
    }

    uint8_t header[44] = {
      0
    };
    audioFile.write(header, 44);

    setupI2S_Input();
    recordWav();
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_0);
    audioFile.close();

    delay(500);

    pinMode(pinPPT, OUTPUT);
    setupI2S_Output();
    playWav(audioFileName); //Повторяем то, что записали
    playWav("/end2.wav"); //Воспроизводим сигнал окончания передачи
    i2s_driver_uninstall(I2S_NUM_0);

    Rec = false;
    pinMode(pinPPT, INPUT);
  }

}