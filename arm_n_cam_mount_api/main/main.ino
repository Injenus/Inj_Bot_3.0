/*
 * 0 - основание руки
 * 1 .. 3 - звенья руки
 * 4 - схват
 * 5 - основание камеры
 * 6 - высота камеры
 */
#include "ServoDriverSmooth.h"
#include "EEPROM.h"

const byte servo_num = 7;
const uint16_t addr = 42;

ServoDriverSmooth servo[servo_num];

struct ServoAngle{
  int pos[servo_num];
};
ServoAngle servoAngle;

uint32_t timer = 0;


uint16_t fletcher16(uint8_t *data, size_t len) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < len; i++) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum2 << 8) | sum1;
}

void setup(){
  Serial.begin(115200);
  EEPROM.get(addr, servoAngle);
  
  for(byte i=0;i++;i<servo_num){
    servo[i].attach(i, 150, 600);
    servo[i].smoothStart();
    servo[i].setAutoDetach(false);
    //servo[i].write(servoAngle.pos[i]);
    servo[i].setMaxAngle(270);
    if (i<5){
      servo[i].setSpeed(360);
      servo[i].setAccel(60);
    }
    else{
      servo[i].setSpeed(360);
      servo[i].setAccel(360);
    }
  }

  pinMode(13, OUTPUT);
  
}

void loop(){
  static uint8_t rBuffer[16]; // Массив для хранения данных
  static size_t index = 0;   // Индекс для записи в буфер
  static int16_t rAngles[7] = {135, 135, 135, 135, 135, 135, 135};
  
  for(byte i=0;i++;i<servo_num){
    servo[i].tickManual(); //tick()
  }


  while (Serial.available() > 0) {
    uint8_t byteReceived = Serial.read();
  
    // Если это первый байт и он равен 'A', начинаем запись в буфер
    if (index == 0 && byteReceived == 'A') {
      rBuffer[index++] = byteReceived;
      Serial.println("Got A");
    }
    // Если уже начали запись, продолжаем добавлять байты в буфер
    else if (index > 0 && index < 16) {
      Serial.println("Inc indx");
      rBuffer[index++] = byteReceived;
  
      // Если буфер заполнен, проверяем данные
      if (index == 16) {
        Serial.println("Got all");
        // Вычисляем контрольную сумму для первых 15 байт
        uint16_t receivedChecksum = (rBuffer[14] | (rBuffer[15] << 8));
        uint16_t calculatedChecksum = fletcher16(rBuffer, 15);
  
        // Сравниваем контрольные суммы
        if (receivedChecksum == calculatedChecksum) {
          // Если контрольная сумма верна, зажигаем светодиод
          digitalWrite(13, HIGH);
          
          // Парсим углы из буфера
          for (size_t i = 0; i < 7; i++) {
            servoAngle.pos[i] = (rBuffer[1 + i * 2] | (rBuffer[2 + i * 2] << 8));
          }
  
          Serial.println("Данные приняты, контрольная сумма верна!");
          for (size_t i = 0; i < 7; i++) {
            Serial.print("Угол ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(servoAngle.pos[i]);
            servo[i].setTargetDeg(servoAngle.pos[i]);
          }
          EEPROM.put(addr, servoAngle);
        } else {
          Serial.println("Ошибка: контрольная сумма неверна.");
          digitalWrite(13, LOW);
        }
        // Сбрасываем индекс буфера для следующего сообщения
        index = 0;
      }
    } else {
      // Если байт первый, но не равен 'A', сбрасываем индекс
      index = 0;
      //Serial.println("Set 0");
    }
  }


}
