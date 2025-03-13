/*
   0 - основание руки
   1 .. 3 - звенья руки
   4 - схват
   5 - основание камеры
   6 - высота камеры

   Ораничения по мкс и углам указаны фактические (реальные).
   При этом на вход должны подаваться углы из дипазона без смещениея (0-255),
   а здесь добавляем смещение и корреткно переводим в мкс.

   Например у нулеового звена фактичсекий диапазон от 0 до 284,
   но принимаем 0-255 и добавляем оффсет 15 согласно таблице.
   Получем, например, для переданного угла 128 значение 143 - действительное среднее положение сервопривода.

   Для сервоприводов камеры всё немного иначе.
   Для сервпривода базы имеем диапазон от -87 до 87, а передам от 0 до 174

   Для отправки углов сначала получаем реальный угол переводом мкс,
   затем вычитаем из угла оффсет и переводим в byte.
   Проверка на байтовый диапазон фактически не трубется, так как мы не можем изначально подать угол больше байта.
*/

#include <ServoDriverSmooth.h>
#include <EEPROM.h>
#include <Wire.h>

#define IDX_Z 3
#define SAMPLES_Z 3
#define SAMPLE_Z_DELAY 2
//#define STAB_DELAY_Z 15
#define STEP_Z 6
//#define STEP_INC 30     // Увеличение шага при успехе
//#define STEP_DEC 15    // Уменьшение шага при неудаче

const byte servoNum = 7;
const uint16_t addr = 42;

ServoDriverSmooth servo[servoNum];

struct ServoSettings {
  int minMcs[servoNum] = {84, 116, 116, 116, 260, 135, 150};
  int maxMcs[servoNum] = {582, 542, 542, 542, 430, 525, 467};
  int minAng[servoNum] = {0, 0, 0, 0, 0, 42, 98};
  int maxAng[servoNum] = {284, 266, 260, 264, 1, 215, 218};
  // int offsetAng[servoNum] = {15, 6, 3, 5, 0, 0, 0};
  int armAcc = 80;
  int armSpeed = 1080;
  int camAcc = 480;
  int camSpeed = 1080;
};
ServoSettings servoSettings;

struct ServoPosAng {
  int pos[servoNum] = {128, 128, 128, 128, 0, 87, 90};
  byte currPos[servoNum] = {0, 0, 0, 0, 0, 0, 0};
};
ServoPosAng servoPosAng, prevServoPosAng;

struct AutoServo{
  int linkDefVal = 128;
  //int init_ang, plus_ang, minus_ang;
  int acc[3] = {0, 0, 0};
  //uint8_t count_init, count_plus, count_minus;
  uint8_t count[3] = {0,0,0};
};
AutoServo autoServo;

uint32_t servoSendTimer = 0;
const uint32_t servoSendPeriod = 21;

const int MPU = 0x68;
int accX, accY, accZ;
bool isPlus;

uint32_t autoServoUpdateTImer = 0;
const uint32_t autoServoUpdatePeriod = 50;

bool i2cBusy = true;

void updateValCount(int* val, uint8_t* counter){
  counter += 1;
  val += mpuReadZ();
}

int mpuReadZ(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accX = (Wire.read() << 8 | Wire.read()); // X-axis value
  accY = (Wire.read() << 8 | Wire.read()); // Y-axis value
  accZ = (Wire.read() << 8 | Wire.read()); // Z-axis value
  Wire.endTransmission(true);
  return accZ;
}

//// Функция усреднения и сглаживания az
//int getAvgAZ() {
//  int sum = 0;
//  for (uint8_t i = 0; i < SAMPLES_Z; i++) {
//    sum += mpuReadZ(); // Замените A0 на нужный пин
//    delay(SAMPLE_Z_DELAY);
//  }
//  int az = sum / SAMPLES_Z;
//  return az;
//}

uint8_t fletcher8(uint8_t *data, size_t len) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < len; i++) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
    //Serial.println(data[i]);
  }
  return (sum1 + sum2) % 255;
}

int anglesToMcs(int angles, byte idx) {
  int from_a_min = servoSettings.minAng[idx];
  int from_a_max = servoSettings.maxAng[idx];
  int to_mcs_min = servoSettings.minMcs[idx];
  int to_mcs_max = servoSettings.maxMcs[idx];
  return map(angles, from_a_min, from_a_max, to_mcs_min, to_mcs_max);
}

int mcsToAngle(int mcs, byte idx) {
  int from_mcs_min = servoSettings.minMcs[idx];
  int from_mcs_max = servoSettings.maxMcs[idx];
  int to_a_min = servoSettings.minAng[idx];
  int to_a_max = servoSettings.maxAng[idx];
  return constrain(map(mcs, from_mcs_min, from_mcs_max, to_a_min, to_a_max), 0, 255);
}

void setup() {
  Serial.begin(115200);
  EEPROM.get(addr, prevServoPosAng);
  
  Wire.begin();
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);
  Wire.endTransmission(true);

  Serial.println(prevServoPosAng.pos[0]);

  servo[0].attach(0, servoSettings.minMcs[0], servoSettings.maxMcs[0]);
  servo[0].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[0], 0));
  servo[0].setAutoDetach(false);
  servo[0].setSpeed(servoSettings.armSpeed);
  servo[0].setAccel(servoSettings.armAcc);

  servo[1].attach(1, servoSettings.minMcs[1], servoSettings.maxMcs[1]);
  servo[1].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[1], 1));
  servo[1].setAutoDetach(false);
  servo[1].setSpeed(servoSettings.armSpeed);
  servo[1].setAccel(servoSettings.armAcc);

  servo[2].attach(2, servoSettings.minMcs[2], servoSettings.maxMcs[2]);
  servo[2].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[2], 2));
  servo[2].setAutoDetach(false);
  servo[2].setSpeed(servoSettings.armSpeed);
  servo[2].setAccel(servoSettings.armAcc);

  servo[3].attach(3, servoSettings.minMcs[3], servoSettings.maxMcs[3]);
  servo[3].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[3], 3));
  servo[3].setAutoDetach(false);
  servo[3].setSpeed(servoSettings.armSpeed);
  servo[3].setAccel(480);

  servo[4].attach(4, servoSettings.minMcs[4], servoSettings.maxMcs[4]);
  servo[4].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[4], 4));
  servo[4].setAutoDetach(false);
  servo[4].setSpeed(servoSettings.armSpeed);
  servo[4].setAccel(servoSettings.armAcc);

  servo[5].attach(8, servoSettings.minMcs[5], servoSettings.maxMcs[5]);
  servo[5].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[5], 5));
  servo[5].setAutoDetach(false);
  servo[5].setSpeed(servoSettings.camSpeed);
  servo[5].setAccel(servoSettings.camAcc);

  servo[6].attach(9, servoSettings.minMcs[6], servoSettings.maxMcs[6]);
  servo[6].writeMicroseconds(anglesToMcs(prevServoPosAng.pos[6], 6));
  servo[6].setAutoDetach(false);
  servo[6].setSpeed(servoSettings.camSpeed);
  servo[6].setAccel(servoSettings.camAcc);

}

void loop() {
  static uint8_t rBuffer[9]; // Массив для хранения принятых данных
  static size_t rIndex = 0;   // Индекс для записи в буфер
  static uint8_t sBuffer[9]; // Массив для хранения отправленных данных
  
  i2cBusy = true;
  servo[0].tick();
  servo[1].tick();
  servo[2].tick();
  servo[3].tick();
  servo[4].tick();
  servo[5].tick();
  servo[6].tick();
  i2cBusy = false;

  while (Serial.available() > 0) {
    uint8_t byteReceived = Serial.read();

    // Если это первый байт и он равен 'A', начинаем запись в буфер
    if (rIndex == 0 && byteReceived == 'A') {
      rBuffer[rIndex++] = byteReceived;
      //      Serial.println("Got A");
    }
    // Если уже начали запись, продолжаем добавлять байты в буфер
    else if (rIndex > 0 && rIndex < 9) {
      rBuffer[rIndex++] = byteReceived;

      // Если буфер заполнен, проверяем данные
      if (rIndex == 9) {
        //        Serial.println("Got all");
        // Вычисляем контрольную сумму для первых 8 байт
        uint8_t receivedChecksum = rBuffer[8];
        uint8_t calculatedChecksum = fletcher8(rBuffer, 8);

        // Сравниваем контрольные суммы
        if (receivedChecksum == calculatedChecksum) {
          // Парсим углы из буфера
          for (size_t i = 0; i < 7; i++) {
            servoPosAng.pos[i] = rBuffer[1 + i];
          }

          //          Serial.println("Right!");
          
          for (size_t i = 0; i < 7; i++) {
            //            Serial.print("mcS ");
            //            Serial.print(i);
            //            Serial.print(": ");
            //            Serial.println(anglesToMcs(servoPosAng.pos[i], i));
            //servo[i].writeMicroseconds(anglesToMcs(servoPosAng.pos[i], i));
            if (i != 3){
              servo[i].setTarget(anglesToMcs(servoPosAng.pos[i], i));
            }
          }
          i2cBusy = false;
          
          if (memcmp(&servoPosAng, &prevServoPosAng, sizeof(ServoPosAng)) != 0) {
            EEPROM.put(addr, servoPosAng);
            memcpy(&prevServoPosAng, &servoPosAng, sizeof(ServoPosAng));
          }

        } else {
          //          Serial.print(receivedChecksum);
          //          Serial.print("  ");
          //          Serial.print(calculatedChecksum);
          //          Serial.println(" Error");
        }
        // Сбрасываем индекс буфера для следующего сообщения
        rIndex = 0;
      }
    } else {
      // Если байт первый, но не равен 'A', сбрасываем индекс
      rIndex = 0;
    }

  }

  if (millis() - servoSendTimer > servoSendPeriod) {
    servoSendTimer = millis();
    sBuffer[0] = 'A';
    for (byte i = 0; i < 7; i++) {
      servoPosAng.currPos[i] = (byte)(mcsToAngle(servo[i].getCurrent(), i));
      sBuffer[i + 1] = servoPosAng.currPos[i];
    }
    sBuffer[8] = fletcher8(sBuffer, 8);
    for (byte i = 0; i < 5; i++) { // чтобы исключить повдреждение и потерю данных
      Serial.write(sBuffer, 9);
    }
    //Serial.println(sBuffer[8]);
  }

  if (millis() - autoServoUpdateTImer > autoServoUpdatePeriod){  // просто обновялем текущее положение актульной инфой от движения, чтобы не уползало
    autoServoUpdateTImer = millis();
    servoPosAng.pos[IDX_Z] = mcsToAngle(servo[IDX_Z].getCurrent(), IDX_Z);
  }

  static int temp_currentPosAng = servoPosAng.pos[IDX_Z];
  if (~i2cBusy){
  
    if (autoServo.count[0] < SAMPLES_Z-1){ // пока не набрали нужно колво семплов - набираем
      updateValCount(&autoServo.acc[0], &autoServo.count[0]);
    }
      
    if (autoServo.count[0] == SAMPLES_Z-1){  // набрали - вычислояем среднее, увеличиваем счётчик, чтобы снвоа сюдап не попасть, обнуляем счётчики плюсов и минусов
      autoServo.acc[0] /= SAMPLES_Z; // получили начальное положение в основном цикле (без блоикровки)
      autoServo.count[0] = SAMPLES_Z; // увеличил чтобы не падать в деление каждыфй раз 
      autoServo.count[1] = 0; // обнулили счетчки для всех сравнителных значенийl
      autoServo.count[2] = 0;
    }
    
    else if (autoServo.count[0] == SAMPLES_Z){ // если счетчик равен числу обнуления + 1, то крутим серву от текущего положения в +10 - положение не сохраняем, неизветсно то ли оно
      servo[IDX_Z].setTarget(anglesToMcs(constrain(prevServoPosAng.pos[IDX_Z]+STEP_Z, servoSettings.minAng[IDX_Z], servoSettings.maxAng[IDX_Z]), IDX_Z));
      autoServo.count[0] = SAMPLES_Z + 1; // увеличиваем счётчик чтобы снова сюда не попатсь
    }
    
    else if (autoServo.count[0] == SAMPLES_Z + 1){ // если мы уже спозиционирвали серву
      if (autoServo.count[1] < SAMPLES_Z-1){
        updateValCount(&autoServo.acc[1], &autoServo.count[1]);
        if (autoServo.count[1] == SAMPLES_Z-1){
          autoServo.acc[1] /= SAMPLES_Z;    // вычислили положение после движения в увеличние
          autoServo.count[1] = SAMPLES_Z; // увеличили чтобы не падать в деление каждыфй раз
          autoServo.count[0] = SAMPLES_Z + 2; // опять увличвли, чтобы не попасть снова

          isPlus = (autoServo.acc[1] - autoServo.acc[0] > 0); // определем, в плюсовую сторону ли нужно двигаться
        }
      }
    }
    
    else if (autoServo.count[0] == SAMPLES_Z + 2){ // попадаем в область корректирвоки угла, коррпектируем один раз, после чего начинаем всё заново
      if (isPlus) { // если нужно добавлять
        servoPosAng.pos[IDX_Z] = temp_currentPosAng + STEP_Z;
      }
      else{
        servoPosAng.pos[IDX_Z] = temp_currentPosAng - STEP_Z;
      }
      servo[IDX_Z].setTarget(anglesToMcs(constrain(servoPosAng.pos[IDX_Z], servoSettings.minAng[IDX_Z], servoSettings.maxAng[IDX_Z]), IDX_Z));
      autoServo.count[0] = 0;
    }
  }

}
