#include <ServoSmooth.h>
#include <EEPROM.h>
#include <Wire.h>

#define ADXL345_ADDR 0x53       // Адрес датчика
#define REG_POWER_CTL 0x2D      // Регистр управления питанием
#define REG_DATA_FORMAT 0x31    // Регистр формата данных
#define REG_DATAX0 0x32         // Начальный регистр данных оси X
#define ACC_AXIC 1  // 0-X, 1-Y, 2-Z

#define SAMPLES 100
#define EEPROM_THRESH 10

const uint16_t eepromAddr = 42;

ServoSmooth servo;

struct ServoSettings {
  int minMcs = 618;
  int maxMcs = 2291;
  int minAng = 0;
  int maxAng = 100;

  int acc = 100;
  int _speed = 1000;
};
ServoSettings servoSettings;

struct Pos {
  uint8_t currAng = 120;
  uint8_t prevAng = 120;
};
Pos pos;

struct AutoDataServo {
  //int init_acc, plus_acc, minus_acc;
  int acc[3] = {0, 0, 0};
  //uint8_t count_init, count_plus, count_minus;
  uint8_t count[3] = {0, 0, 0};
};
AutoDataServo autoDataServo;

struct PersistentPos {
  uint32_t version;  // 4 байта
  Pos data;          // 2 байта
};

const uint16_t EEPROM_SIZE = EEPROM.length();
const uint16_t RECORD_SIZE = sizeof(PersistentPos);
const uint16_t MAX_RECORDS = EEPROM_SIZE / RECORD_SIZE;


uint32_t hardUpdPosTimer = 0;
uint32_t hardUpdPosPeriod = 500;

uint32_t mainTimer = 0;
uint32_t mainPeriod = 2;

int16_t accBuff[SAMPLES];
int16_t accVal = 0;
int16_t prevAccVal = 0;

void writePosition(const Pos& pos) {
  static uint32_t current_version = 0;
  
  // Читаем последнюю версию
  PersistentPos last;
  EEPROM.get(0, last);
  for(uint16_t i=1; i<MAX_RECORDS; i++) {
      PersistentPos tmp;
      EEPROM.get(i*RECORD_SIZE, tmp);
      if(tmp.version > last.version) last = tmp;
  }
  
  // Вычисляем новую версию и позицию
  current_version = last.version + 1;
  uint16_t write_addr = (current_version % MAX_RECORDS) * RECORD_SIZE;
  
  // Записываем данные
  PersistentPos new_record = {current_version, pos};
  EEPROM.put(write_addr, new_record);
}

Pos readLastPosition() {
  PersistentPos last = {0};
  for(uint16_t i=0; i<MAX_RECORDS; i++) {
      PersistentPos tmp;
      EEPROM.get(i*RECORD_SIZE, tmp);
      if(tmp.version > last.version) last = tmp;
  }
  return last.data;
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
//  Serial.print("X,Y,Z: ");
//  Serial.print(xRaw);
//  Serial.print(',');
//  Serial.print(yRaw);
//  Serial.print(',');
//  Serial.println(zRaw);

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

int16_t PID(const float& currVal) {
    static float I = 0;
    static float prevError = 60;
    static uint16_t target = 0;

    static float P_koeff = 0.05;
    static float I_koeff = 0.6;
    static float D_koeff = 0.01;
    static float dt = SAMPLES*mainPeriod/1000.;
    static float maxIntegral = 100000.;

    float error = -(currVal - target);

    float P = P_koeff * error;
    I += I_koeff * error * dt;
    I = constrain(I, -maxIntegral, maxIntegral);
    float D = D_koeff * (error - prevError) / dt;

    Serial.println(P);
    Serial.println(I);
    Serial.println(D);
    

    float outputVal = P + I + D;
    
    int16_t servoAngle = (int16_t)(constrain(outputVal, servoSettings.minAng, servoSettings.maxAng)- 40);
    prevError = error;

    //if (is_neg) servoAngle -= (int16_t)target / 1.5;
    
    return servoAngle;
}

int initError = 0;
void setup() {

  Serial.begin(115200);
  pos = readLastPosition();
  Wire.begin();
  Wire.setClock(400000UL);

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

  initError = pos.currAng;

  servo.attach(9, servoSettings.minMcs, servoSettings.maxMcs);
  servo.writeMicroseconds(anglesToMcs(pos.currAng));
  servo.setAutoDetach(false);
  servo.setSpeed(servoSettings._speed);
  servo.setAccel(servoSettings.acc);

}

void loop() {

  static bool isEepromAcc = false;
  static uint16_t counter = 0;
  static int16_t pid = 0;
  static int16_t iterCounter = 0;
  
  servo.tick();

  static uint32_t last_save = 0;
    if(millis() - last_save >= 5000) {
        last_save = millis();
        Pos current_pos;
        current_pos.currAng = mcsToAngle(servo.getCurrent());
        current_pos.prevAng = current_pos.currAng;
        writePosition(current_pos);
    }

  if (millis() - mainTimer > mainPeriod){
    mainTimer = millis();
    if (counter == SAMPLES){
      counter = 0;
      accVal = (float)accVal / SAMPLES;

      
      if (iterCounter < 8){
        //Serial.println(-initError);
        pid = PID(-initError);
        pid = initError;
        //Serial.println("IIIIIIINNNIIIIIIIIITTTTTTTTT");
        iterCounter++;
      }
      else{
        pid = PID(accVal);
      }
      //Serial.println(pid);
      
      servo.setTarget(anglesToMcs(pid));

      Serial.println("aver, pid,  curr");
      Serial.print(accVal);
      Serial.print(",  ");
      Serial.print(pid);
      Serial.print(",  ");
      Serial.println(mcsToAngle(servo.getCurrent()));
      
      prevAccVal = accVal;
      accVal = 0;
    }
    else{
      counter++;
      accVal += getAccData();
    }
  }

}
