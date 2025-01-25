/*
   0 - основание руки
   1 .. 3 - звенья руки
   4 - схват
   5 - основание камеры
   6 - высота камеры
*/
#include "ServoDriverSmooth.h"
#include <EEPROM.h>

const byte servoNum = 7;
const uint16_t addr = 42;

ServoDriverSmooth servo[servoNum];

struct ServoSettings {
  int min_mcs[servoNum] = {84, 116, 116, 116, 300, 135, 150};
  int max_mcs[servoNum] = {582, 542, 542, 542, 492, 525, 467};
  int min_angles[servoNum] = {0, 0, 0, 0, 0, 0, 0};
  int max_angles[servoNum] = {270, 270, 270, 270, 100, 180, 100};
};
ServoSettings servoSettings;

struct ServoPosAng {
  //int pos[servoNum];
  int pos[servoNum] = {135, 135, 135, 135, 120, 135, 100};
};
ServoPosAng servoPosAng;

uint32_t timer = 0;


uint8_t fletcher8(uint8_t *data, size_t len) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (size_t i = 0; i < len; i++) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum1 + sum2) % 255;
}

int anglesToMcs(int angles, byte idx) {
  int from_a_min = servoSettings.min_angles[idx];
  int from_a_max = servoSettings.max_angles[idx];
  int to_mcs_min = servoSettings.min_mcs[idx];
  int to_mcs_max = servoSettings.max_mcs[idx];
  return map(angles, from_a_min, from_a_max, to_mcs_min, to_mcs_max);
}

void setup() {
  Serial.begin(115200);
  EEPROM.get(addr, servoPosAng);
  Serial.println(servoPosAng.pos[0]);

  servo[0].attach(0, servoSettings.min_mcs[0], servoSettings.max_mcs[0]);
  servo[0].writeMicroseconds(anglesToMcs(servoPosAng.pos[0], 0));
  servo[0].setAutoDetach(false);
  servo[0].setSpeed(240);
  servo[0].setAccel(40);

  servo[1].attach(1, servoSettings.min_mcs[1], servoSettings.max_mcs[1]);
  servo[1].writeMicroseconds(anglesToMcs(servoPosAng.pos[1], 1));
  servo[1].setAutoDetach(false);
  servo[1].setSpeed(240);
  servo[1].setAccel(40);

  servo[2].attach(2, servoSettings.min_mcs[2], servoSettings.max_mcs[2]);
  servo[2].writeMicroseconds(anglesToMcs(servoPosAng.pos[2], 2));
  servo[2].setAutoDetach(false);
  servo[2].setSpeed(240);
  servo[2].setAccel(40);

  servo[3].attach(3, servoSettings.min_mcs[3], servoSettings.max_mcs[3]);
  servo[3].writeMicroseconds(anglesToMcs(servoPosAng.pos[3], 3));
  servo[3].setAutoDetach(false);
  servo[3].setSpeed(240);
  servo[3].setAccel(40);

  servo[4].attach(4, servoSettings.min_mcs[4], servoSettings.max_mcs[4]);
  servo[4].writeMicroseconds(anglesToMcs(servoPosAng.pos[4], 4));
  servo[4].setAutoDetach(false);
  servo[4].setSpeed(240);
  servo[4].setAccel(40);

  servo[5].attach(5, servoSettings.min_mcs[5], servoSettings.max_mcs[5]);
  servo[5].writeMicroseconds(anglesToMcs(servoPosAng.pos[5], 5));
  servo[5].setAutoDetach(false);
  servo[5].setSpeed(360);
  servo[5].setAccel(360);

  servo[6].attach(6, servoSettings.min_mcs[6], servoSettings.max_mcs[6]);
  servo[6].writeMicroseconds(anglesToMcs(servoPosAng.pos[6], 6));
  servo[6].setAutoDetach(false);
  servo[6].setSpeed(360);
  servo[6].setAccel(360);

}

void loop() {
  static uint8_t rBuffer[16]; // Массив для хранения данных
  static size_t index = 0;   // Индекс для записи в буфер

//  for (byte i = 0; i++; i < servoNum) {
//    servo[i].tick(); //tick()
//  }
  servo[0].tick();
  servo[1].tick();
  servo[2].tick();
  servo[3].tick();
  servo[4].tick();
  servo[5].tick();
  servo[6].tick();

  while (Serial.available() > 0) {
    uint8_t byteReceived = Serial.read();

    // Если это первый байт и он равен 'A', начинаем запись в буфер
    if (index == 0 && byteReceived == 'A') {
      rBuffer[index++] = byteReceived;
      Serial.println("Got A");
    }
    // Если уже начали запись, продолжаем добавлять байты в буфер
    else if (index > 0 && index < 9) {
      rBuffer[index++] = byteReceived;
    
      // Если буфер заполнен, проверяем данные
      if (index == 9) {
        Serial.println("Got all");
        // Вычисляем контрольную сумму для первых 8 байт
        uint8_t receivedChecksum = rBuffer[8];
        uint8_t calculatedChecksum = fletcher8(rBuffer, 8);
    
        // Сравниваем контрольные суммы
        if (receivedChecksum == calculatedChecksum) {
          // Парсим углы из буфера
          for (size_t i = 0; i < 7; i++) {
            servoPosAng.pos[i] = rBuffer[1 + i];
          }
    
          Serial.println("Right!");
          for (size_t i = 0; i < 7; i++) {
            Serial.print("mcS ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(anglesToMcs(servoPosAng.pos[i], i));
            servo[i].writeMicroseconds(anglesToMcs(servoPosAng.pos[i], i));
          }
          EEPROM.put(addr, servoPosAng);
        } else {
          Serial.print(receivedChecksum);
          Serial.print("  ");
          Serial.print(calculatedChecksum);
          Serial.println(" Error");
        }
        // Сбрасываем индекс буфера для следующего сообщения
        index = 0;
      }
    } else {
      // Если байт первый, но не равен 'A', сбрасываем индекс
      index = 0;
    }

  }


}
