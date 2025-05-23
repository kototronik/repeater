# Репитер на основе ESP32



Репитер на базе микроконтроллера ESP32, предназначенный для расширения зоны покрытия радиосвязи. Позволяет принимать и передавать аналоговые FM сигналы, ретранслируя их и увеличивая таким образом эффективное расстояние связи при использовании любой радиостанции с разъемом Kenwood в качестве приемо-передатчика.

**Основные возможности:**

* Повторение аналоговых FM сигналов.
* Поддержка радиостанций с интерфейсом Kenwood (стандартный 6-пиновый разъем).
* Низкое энергопотребление, что делает возможным использование с портативными источниками питания.
* Гибкость использования как в стационарных условиях (например, для покрытия определенной территории), так и в мобильных (например, в полевых условиях с повербанком).

**Основа проекта:**

Данный проект основан на работе [Печенова Александра (студент ФГБОУ ВО "ДОННТУ" г. Донецк)](https://disk.yandex.ru/d/aPEOfnw8R4aH_Q).

**Список основных компонентов:**

* Микроконтроллер ESP32.
* Радиостанция с разъемом Kenwood (например, Baofeng, Kenwood, Quansheng и другие модели с совместимым разъемом).
* Источник питания 5В (например, повербанк + USB-адаптер).
* Компоненты для развязки. См. схему подключения.

* *(В будущем)* Внешний ЦАП для улучшения качества передаваемого аудиосигнала *(опционально)*.


**Схема подключения разъема Kenwood:**

![Схема подключения разъема Kenwood](https://github.com/kototronik/repeater/blob/main/images/scheme.png?raw=true)


**Схема подключения SD-карты:**

| SD Card Module     | ESP32     |
|--------------------|-----------|
| CS                 | GPIO 5    |
| SCK                | GPIO 18   |
| MOSI (SDI)         | GPIO 23   |
| MISO (SDO)         | GPIO 19   |
| VCC                | 3.3V      |
| GND                | GND       |

**Текущий статус проекта:**

* **Версия с использованием встроенного ЦАП:** Находится на стадии активного тестирования и доработки. Выявлены некоторые недоработки, влияющие на качество звука (искажения звука, посторонние звуки), которые планируется устранить путем оптимизации кода, фильтрации сигналов и экспериментов со встроенным I2S, вместо прямой записи ЦАП. Стабильность работы в целом удовлетворительная.

* **Версия с внешним ЦАП:** Разработка находится на начальной стадии. Ожидается доставка внешнего ЦАП для проведения первых тестов и оценки улучшения качества звука. После получения ЦАП планируется разработка схемы подключения и написания прошивки.

**Планы на будущее:**

* **Доработка версии со встроенным ЦАП:**
    * Оптимизация работы записи и воспроизведения аудиоданных для достижения лучшей разборчивости речи и минимизации искажений.
* **Создание версии с внешним ЦАП:**
    * Разработка схемы подключения внешнего ЦАП к ESP32.
    * Написание программного обеспечения для работы с внешним ЦАП.
    * Оптимизация работы с внешним ЦАП для достижения наилучшего качества передачи.

## Корпус
Файлы 3D-моделей корпуса и крышки доступны в папке `3D` в формате `.stp`.