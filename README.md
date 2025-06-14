# Репитер на базе ESP32

Проект репитера на основе микроконтроллера **ESP32**, предназначенного для расширения зоны аналоговой FM-связи. Устройство принимает и повторяет сигналы, увеличивая эффективное расстояние связи при использовании радиостанций с разъёмом **Kenwood**.

---

## Основные возможности

- Повторение аналоговых FM-сигналов  
- Совместимость с радиостанциями с 6-пиновым разъёмом **Kenwood** (Baofeng, Kenwood, Quansheng и др.)  
- Подходит для стационарного и мобильного использования  

---

## Основные компоненты

- **Микроконтроллер** ESP32  
- **Радиостанция** с разъёмом Kenwood  
- **Источник питания** 5В  
- **Схема согласования и развязки аудиосигналов** (см. ниже)  
- **Внешний ЦАП** PCM5102A *(опционально)*  

---

## Схема подключения разъёма Kenwood

![Схема подключения разъема Kenwood](https://github.com/kototronik/repeater/blob/main/images/scheme.png?raw=true)

> **Примечание:** в версии с внешним ЦАПом пин **PTT = GPIO 32**

---

## Подключение SD-карты

| **SD-карта**     | **ESP32**   |
|------------------|-------------|
| CS               | GPIO 5      |
| SCK              | GPIO 18     |
| MOSI (SDI)       | GPIO 23     |
| MISO (SDO)       | GPIO 19     |
| VCC              | 3.3В *(или 5В)* |
| GND              | GND         |

---

## Подключение внешнего ЦАП (PCM5102A)

| **ЦАП PCM5102A** | **ESP32**   |
|------------------|-------------|
| DIN (DOUT)       | GPIO 25     |
| BCK (BCLK)       | GPIO 27     |
| LCK (LRCK)       | GPIO 26     |
| VCC              | 3.3В        |
| GND              | GND         |

---

## Статус проекта

- **Встроенный ЦАП ESP32**  
  Работает, но присутствуют искажения и посторонние шумы. При слабом входном сигнале — плохая разборчивость речи.  
  **Доработка не планируется.**

- **Внешний ЦАП (PCM5102A)**  
  Активно тестируется. При хорошем экранировании — качественное звучание.  
  **В разработке.**

---

## Планы на будущее

- Создание графических схем подключения 
- Дальнейшее тестирование  
- Исправление обнаруженных проблем  

---

## Корпус

3D-модели корпуса и крышки находятся в папке `3D` (формат `.stp`).

---

## Источник

Проект основан на работе [Печенова Александра (ФГБОУ ВО "ДОННТУ", г. Донецк)](https://disk.yandex.ru/d/aPEOfnw8R4aH_Q).
