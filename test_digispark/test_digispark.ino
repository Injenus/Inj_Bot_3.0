#include <Wire.h>
#include <Servo.h>

// Адрес датчика ADXL345
#define ADXL345_ADDR 0x53

// Регистры датчика
#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  Wire.begin();
  
  // Проверка соединения
  if (readRegister(ADXL345_REG_DEVID) != 0xE5) {
    //Serial.println("ADXL345 not found!");
    while(1);
  }
  
  // Включение измерения
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);
  
  // Настройка диапазона
  writeRegister(ADXL345_REG_DATA_FORMAT, 0x08);
  
  delay(100);
  digitalWrite(1, LOW);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}

void loop() {
  int16_t x, y, z;
  
  // Чтение данных акселерометра
  readAccel(&x, &y, &z);
  

  if (abs(z) < 10){
    digitalWrite(1, LOW);
  }
  else{
    digitalWrite(1, HIGH);
  }

  
  delay(200);
}

void readAccel(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(ADXL345_REG_DATAX0);
  Wire.endTransmission(false);
  
  Wire.requestFrom(ADXL345_ADDR, 6);
  *x = (Wire.read() | (Wire.read() << 8));
  *y = (Wire.read() | (Wire.read() << 8));
  *z = (Wire.read() | (Wire.read() << 8));
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(ADXL345_ADDR, 1);
  return Wire.read();
}
