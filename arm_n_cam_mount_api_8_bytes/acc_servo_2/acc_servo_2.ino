#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>

#define SERVO_PIN 9
#define CALIBRATION_DELAY 500  // Время на перемещение сервы
#define FILTER_SAMPLES 10      // Усреднение показаний
#define ACC_AXIC 1

#define ADXL345_ADDR 0x53       // Адрес датчика
#define REG_POWER_CTL 0x2D      // Регистр управления питанием
#define REG_DATA_FORMAT 0x31    // Регистр формата данных
#define REG_DATAX0 0x32         // Начальный регистр данных оси X
#define ACC_AXIC 1  // 0-X, 1-Y, 2-Z


Servo servo;
int16_t rawAccel;             // "Сырое" значение акселерометра
float filteredAccel = 0;       // Отфильтрованное значение
float target = 0;              // Целевое значение (0)
float currentAngle = 120;      // Текущий угол сервы
float Kp = 0.15;              // Коэффициент пропорциональности
int8_t correctionDirection = 1; // Направление коррекции (1 или -1)
uint32_t control_timer = 0;

struct ServoSettings {
    int minMcs = 618;
    int maxMcs = 2291;
    int minAng = 0;
    int maxAng = 240;
  };
  ServoSettings servoSettings;


  // Динамический расчёт размера блока под конкретную EEPROM
const int RECORD_SIZE = sizeof(uint32_t) + sizeof(byte); // 5 байт (4 + 1)
const int BLOCK_SIZE = EEPROM.length() / RECORD_SIZE;     // Для Nano: 1024 / 5 = 204 записей

struct Record {
  uint32_t counter;  // 32-битный счётчик
  byte data;         // Сохраняемый байт
};

// Поиск последней валидной записи
int findLatestRecordIndex() {
    int latestIndex = 0;
    uint32_t maxCounter = 0;
    for (int i = 0; i < BLOCK_SIZE; i++) {
      Record r;
      EEPROM.get(i * RECORD_SIZE, r);
      if (r.counter != 0xFFFFFFFF && r.counter > maxCounter) { // Игнорируем "пустые" ячейки
        maxCounter = r.counter;
        latestIndex = i;
      }
    }
    return latestIndex;
  }
  
  // Запись с циклическим перезаписом блоков
  void writeNextRecord(byte value) {
    static int currentIndex = 0;
    static uint32_t counter = 0;
  
    // Инициализация счётчика из EEPROM
    if (counter == 0) {
      for (int i = 0; i < BLOCK_SIZE; i++) {
        Record r;
        EEPROM.get(i * RECORD_SIZE, r);
        if (r.counter > counter) counter = r.counter;
      }
      counter++; // Следующее значение
    }
  
    Record r = {counter++, value};
    EEPROM.put(currentIndex * RECORD_SIZE, r);
  
    currentIndex = (currentIndex + 1) % BLOCK_SIZE; // Цикл по EEPROM
  }


  int16_t getAccData() {
    static int xRaw, yRaw, zRaw;
    // Чтение данных всех осей
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(REG_DATAX0);      // Указываем начальный регистр данных
    Wire.endTransmission(false);
  
    // Запрашиваем 6 байтов (по 2 на каждую ось)
    Wire.requestFrom(ADXL345_ADDR, 6, true);
  
    if (Wire.available() >= 6) {
      xRaw = Wire.read() | (Wire.read() << 8);
      yRaw = Wire.read() | (Wire.read() << 8);
      zRaw = Wire.read() | (Wire.read() << 8);
    }
  
    switch (ACC_AXIC) {
      case 0:
        return xRaw;
      //        break;
      case 1:
        return yRaw;
      //        break;
      case 2:
        return zRaw;
      //        break;
      default:
        return 0;
        //        break;
    }
  }

  int anglesToMcs(int angles) {
    int from_a_min = servoSettings.minAng;
    int from_a_max = servoSettings.maxAng;
    int to_mcs_min = servoSettings.minMcs;
    int to_mcs_max = servoSettings.maxMcs;
    return map(angles, from_a_min, from_a_max, to_mcs_min, to_mcs_max);
  }
  
  int mcsToAngle(int mcs) {
    int from_mcs_min = servoSettings.minMcs;
    int from_mcs_max = servoSettings.maxMcs;
    int to_a_min = servoSettings.minAng;
    int to_a_max = servoSettings.maxAng;
    return constrain(map(mcs, from_mcs_min, from_mcs_max, to_a_min, to_a_max), 0, 255);
  }


  // Инициализация ШИМ на пине 9 (таймер 1)
void initPWM(int init_width) {
    DDRB |= (1 << PB1);    // Пин 9 (OC1A) как выход
    TCCR1A = (1 << COM1A1) | (1 << WGM11); // Неинверсный ШИМ, режим 14 (Fast PWM, TOP=ICR1)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Режим 14, предделитель 8
    ICR1 = 20000;          // Период по умолчанию 20 мс (50 Гц)
    OCR1A = init_width;          // Начальная ширина импульса
  }
  
  // Установка ширины импульса в микросекундах (500-2500)
  void setPWMPulse(uint16_t microseconds) {
    OCR1A = constrain(microseconds, 500, 2500);
  }


void setup() {
    Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.write(currentAngle);
  determineDirection();        // Определяем направление коррекции
}

void loop() {
    if (millis() - control_timer > 10) {
        control_timer = millis();
        updateAccel();               // Обновляем данные акселерометра
        float error = filteredAccel - target;

        // Пропорциональное управление
        currentAngle += Kp * error * correctionDirection;

        // Ограничение диапазона
        currentAngle = constrain(currentAngle, 0, 240);

        setPWMPulse(currentAngle);
    }
}



// Определение направления коррекции
int8_t determineDirection() {
  float baseError = getFilteredAccel();
  int ang_val = 10;
  
  // Тест в положительном направлении
  setPWMPulse(anglesToMcs(120 + ang_val));
  delay(CALIBRATION_DELAY);
  float errorPlus = getFilteredAccel();
  
  // Тест в отрицательном направлении
  setPWMPulse(120 - ang_val);
  delay(CALIBRATION_DELAY);
  float errorMinus = getFilteredAccel();
  
  // Возврат в исходное положение
  setPWMPulse(120);
  delay(CALIBRATION_DELAY);
  
  // Выбор направления с меньшей ошибкой
  if (abs(errorPlus) < abs(baseError) || abs(errorMinus) < abs(baseError)) {
    correctionDirection = (abs(errorPlus) < abs(errorMinus)) ? 1 : -1;
  } else {
    correctionDirection = 1; // По умолчанию, если не удалось определить
  }
  Serial.println(correctionDirection);
  return;
}

// Чтение и фильтрация данных акселерометра
void updateAccel() {
  int32_t sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    // Замените эту строку на реальное чтение данных вашего акселерометра!
    // Пример для аналогового датчика:
    // rawAccel = analogRead(A0) - 512;
    // Для цифрового (например, ADXL345):
    // readAccelSensor(&rawAccel);
    sum += rawAccel;
  }
  filteredAccel = sum / (float)FILTER_SAMPLES;
}

float getFilteredAccel() {
  updateAccel();
  return filteredAccel;
}
