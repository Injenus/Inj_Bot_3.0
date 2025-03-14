
#include <ServoSmooth.h>
#include <EEPROM.h>
#include <Wire.h>

#define ADXL345_ADDR 0x53       // Адрес датчика
#define REG_POWER_CTL 0x2D      // Регистр управления питанием
#define REG_DATA_FORMAT 0x31    // Регистр формата данных
#define REG_DATAX0 0x32         // Начальный регистр данных оси X
#define ACC_AXIC 1  // 0-X, 1-Y, 2-Z

#define SAMPLES 10
#define RELEV_NUM 3
#define STEP 3
#define EEPROM_THRESH 10

int accBuffer[SAMPLES];
int idxBuffer = 0;          // Текущая позиция для записи
bool bufferFilled = false; // Флаг заполнения буфера

const uint16_t eepromAddr = 42;

ServoSmooth servo;

struct ServoSettings {
  int minMcs = 116;
  int maxMcs = 542;
  int minAng = 0;
  int maxAng = 264;

  int acc = 350;
  int _speed = 1080;
};
ServoSettings servoSettings;

struct Pos {
  int currAng = 132;
  int prevAng = 132;
  int targetAng = 132;
};
Pos pos;

struct AutoDataServo {
  //int init_acc, plus_acc, minus_acc;
  int acc[3] = {0, 0, 0};
  //uint8_t count_init, count_plus, count_minus;
  uint8_t count[3] = {0, 0, 0};
};
AutoDataServo autoDataServo;

uint16_t hardUpdPosTimer = 0;
uint16_t hardUpdPosPeriod = 50;
bool isPlus;


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

void updateValCount(int& acc, uint8_t& counter) {
  if (counter >= SAMPLES - RELEV_NUM) acc += getAccData();
  counter++;
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


void setup() {
  if (RELEV_NUM > SAMPLES) {
    pinMode(13, OUTPUT);
    while (1) {
      digitalWrite(13, 1);
      delay(200);
      digitalWrite(13, 0);
      delay(350);
    }
  }
  Serial.begin(115200);
  EEPROM.get(eepromAddr, pos);
  Wire.begin();

  // Активация датчика
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_POWER_CTL);
  Wire.write(0x08);            // Режим измерения
  Wire.endTransmission();

  // Настройка диапазона (±2g)
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATA_FORMAT);
  Wire.write(0x00);            // ±2g (значение по умолчанию)
  Wire.endTransmission();

  servo.attach(9, servoSettings.minMcs, servoSettings.maxMcs);
  servo.writeMicroseconds(anglesToMcs(pos.prevAng));
  servo.setAutoDetach(false);
  servo.setSpeed(servoSettings._speed);
  servo.setAccel(servoSettings.acc);
}

void loop() {
  servo.tick();


  if (millis() - hardUpdPosTimer > hardUpdPosPeriod) {
    hardUpdPosTimer = millis();
    pos.currAng = mcsToAngle(servo.getCurrent());
    if (abs(pos.currAng - pos.prevAng) > EEPROM_THRESH) {
      pos.prevAng = pos.currAng;
      EEPROM.put(eepromAddr, pos);
    }
  }


  if (autoDataServo.count[0] < SAMPLES) { // пока не набрали нужно колво семплов - набираем
    updateValCount(autoDataServo.acc[0], autoDataServo.count[0]);
  }

  if (autoDataServo.count[0] == SAMPLES) { // набрали - вычислояем среднее, увеличиваем счётчик, чтобы снвоа сюдап не попасть, обнуляем счётчики плюсов и минусов
    autoDataServo.acc[0] /= RELEV_NUM; // получили начальное положение в основном цикле (без блоикровки)
    autoDataServo.count[0] = SAMPLES + 1; // увеличил чтобы не падать в деление каждыфй раз
    autoDataServo.count[1] = 0; // обнулили счетчки для всех сравнителных значенийl
    //autoDataServo.count[2] = 0;
  }

  else if (autoDataServo.count[0] == SAMPLES + 1) { // если счетчик равен числу обнуления + 1, то крутим серву от текущего положения в +10 - положение не сохраняем, неизветсно то ли оно
    servo.setTarget(anglesToMcs(constrain(pos.currAng + STEP, servoSettings.minAng, servoSettings.maxAng)));
    autoDataServo.count[0] = SAMPLES + 2; // увеличиваем счётчик чтобы снова сюда не попатсь
  }

  else if (autoDataServo.count[0] == SAMPLES + 2) { // если мы уже спозиционирвали серву
    if (autoDataServo.count[1] < SAMPLES) {
      updateValCount(autoDataServo.acc[1], autoDataServo.count[1]);
      if (autoDataServo.count[1] == SAMPLES) {
        autoDataServo.acc[1] /= RELEV_NUM;    // вычислили положение после движения в увеличние
        autoDataServo.count[1] = SAMPLES + 1; // увеличили чтобы не падать в деление каждыфй раз
        autoDataServo.count[0] = SAMPLES + 3; // опять увличвли, чтобы не попасть снова

        isPlus = (autoDataServo.acc[1] - autoDataServo.acc[0] > 0); // определем, в плюсовую сторону ли нужно двигаться
      }
    }
  }

  else if (autoDataServo.count[0] == SAMPLES + 3) { // попадаем в область корректирвоки угла, коррпектируем один раз, после чего начинаем всё заново
    if (isPlus) { // если нужно добавлять
      //pos.targetAng = pos.currAng; //+ STEP;
    }
    else {
      //pos.targetAng = pos.currAng - 2 * STEP;
      servo.setTarget(anglesToMcs(constrain(pos.currAng - 2 * STEP, servoSettings.minAng, servoSettings.maxAng)));
    }
    autoDataServo.count[0] = 0;
    autoDataServo.acc[0] = 0;
    autoDataServo.acc[1] = 0;
  }

}
