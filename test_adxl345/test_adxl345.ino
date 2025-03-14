#include <Wire.h>

#define ADXL345_ADDR 0x53       // Адрес датчика
#define REG_POWER_CTL 0x2D      // Регистр управления питанием
#define REG_DATA_FORMAT 0x31    // Регистр формата данных
#define REG_DATAX0 0x32         // Начальный регистр данных оси X

void setup() {
  Serial.begin(115200);
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
}

void loop() {
  // Чтение данных всех осей
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATAX0);      // Указываем начальный регистр данных
  Wire.endTransmission(false);
  
  // Запрашиваем 6 байтов (по 2 на каждую ось)
  Wire.requestFrom(ADXL345_ADDR, 6, true);
  
  if (Wire.available() >= 6) {
    // Чтение данных для оси X
    int16_t xRaw = Wire.read() | (Wire.read() << 8);
    // Чтение данных для оси Y
    int16_t yRaw = Wire.read() | (Wire.read() << 8);
    // Чтение данных для оси Z
    int16_t zRaw = Wire.read() | (Wire.read() << 8);
    
    // Вывод данных
    Serial.print("X: ");
    Serial.print(xRaw);
    Serial.print(" | Y: ");
    Serial.print(yRaw);
    Serial.print(" | Z: ");
    Serial.println(zRaw);
  }
  
  delay(5);
}
