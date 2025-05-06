#include <SD.h>

#include <FS.h>

#define dt 110 // Период квантования ЦАП, мкс
#define pinLED 2 // Встроенный светодиод
#define pinLED2 13 // Внешний светодиод
#define pinADC 32 // АЦП
#define pinDAC 25 // ЦАП
#define pinPPT 27 // Программный аналог кнопки PTT
#define pinSQL 12 // Триггер записи (радиостанция, при открытии шумоподавителя, посылает постоянный ток на этот пин)
#define morseFreq 1000 // Несущая частота морзянки (завершение передачи будет на этой частоте)

long T_dt; // Таймер воспроизведения
bool Rec = false; // Флаг записи

File audioFile;

void setup() {
  setCpuFrequencyMhz(40); //Включение экономии энергии

  pinMode(pinLED, OUTPUT);
  pinMode(pinLED2, OUTPUT);

  digitalWrite(pinLED2, 1);
  delay(1000);
  digitalWrite(pinLED2, 0);

  pinMode(pinPPT, INPUT);
  pinMode(pinSQL, INPUT);

  Serial.begin(9600);
  Serial.println("start");

  if (!SD.begin()) {
    Serial.println("Ошибка инициализации SD-карты!");
    return;
  }
  Serial.println("SD карта инициализирована");
}

void playMorseSymbol(bool isDot) {
  unsigned long duration = isDot ? 50 : 150; // 50 мс для точки, 150 мс для тире
  unsigned long startTime = millis();

  // Генерация сигнала с частотой morseFreq Гц (с использованием прямоугольного сигнала через ЦАП)
  while (millis() - startTime < duration) {
    dacWrite(pinDAC, 255); // Выводим высокий уровень (максимальное значение для аналогового сигнала, доступного для встроенного АЦП)
    delayMicroseconds(500); // Высокий уровень для полуволны
    dacWrite(pinDAC, 0); // Выводим низкий уровень
    delayMicroseconds(500); // Низкий уровень для полуволны
  }

  delay(50); // Интервал между символами
}

void playMorseCode() {
  // Морзянка для "*-*-*" (с быстрым интервалом между символами)
  playMorseSymbol(true);  // *
  playMorseSymbol(false); // -
  playMorseSymbol(true);  // *
  playMorseSymbol(false); // -
  playMorseSymbol(true);  // *
}

void loop() {
  bool SQL = digitalRead(pinSQL);
  digitalWrite(pinLED2, SQL); // Индикация записи

  // Начало записи
  if (SQL && !Rec) {
    Rec = true;
    setCpuFrequencyMhz(240); // Выход из режима экономии
    T_dt = micros();

    // Открываем файл для записи
    audioFile = SD.open("/audio.dat", FILE_WRITE);
    if (!audioFile) {
      Serial.println("Ошибка открытия файла для записи!");
      Rec = false;
      return;
    }

    Serial.println("Начало записи...");
  }

  // Запись данных на SD пока активен pinSQL
  unsigned long silenceStart = 0;

  while (Rec) {
    bool currentSQL = digitalRead(pinSQL);

    if (currentSQL) {
      silenceStart = 0; // сбрасываем таймер тишины

      if (micros() - T_dt >= dt) {
        T_dt = micros();
        byte value = analogRead(pinADC) / 16; // Преобразуем 12-бит в 8-бит
        audioFile.write(value);
      }
    } else {
      if (silenceStart == 0) {
        silenceStart = millis(); // фиксируем момент начала тишины
      } else if (millis() - silenceStart >= 1000) {
        break; // тишина более 1 секунды - прерываем запись
      }
    }
  }

  // Завершение записи и начало воспроизведения
  if (Rec && digitalRead(pinSQL) == 0) {
    Rec = false;
    audioFile.close();
    Serial.println("Запись завершена");
    delay(1000);

    // Переход к передаче
    pinMode(pinPPT, OUTPUT);
    Serial.println("Передача...");

    File playFile = SD.open("/audio.dat");
    if (playFile) {
      unsigned long T_play = micros();
      while (playFile.available()) {
        byte val = playFile.read();
        dacWrite(pinDAC, val);

        // Воспроизведение
        while (micros() - T_play < dt) {
          // ждём до наступления следующего квантования
        }
        T_play += dt;

        if (playFile.position() % 1000 == 0)
          digitalWrite(pinLED2, !digitalRead(pinLED2));
      }
      playFile.close();
      playMorseCode(); // Воспроизведение морзянки после аудио
    }

    digitalWrite(pinLED2, 0);
    delay(500);
    pinMode(pinPPT, INPUT);
    setCpuFrequencyMhz(40);
    Serial.println("Передача завершена");

  }
}
