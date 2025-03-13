#include <Wire.h>

// Настройки ADXL345
#define DEVICE_ADDR 0x53 // Адрес I2C (0x53 или 0x1D)
#define REG_POWER_CTL 0x2D // Регистр включения питания
#define REG_DATA_FORMAT 0x31 // Регистр настройки диапазона
#define REG_DATAZ0 0x36 // Регистр данных Z (младший байт)
#define REG_DATAZ1 0x37 // Регистр данных Z (старший байт)

// Коэффициент преобразования (зависит от диапазона)
float scale_factor = 9.81 / 256; // Для диапазона ±2g (256 LSB/g)

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Инициализация I2C

  // Настройка датчика
  writeRegister(REG_POWER_CTL, 0x08); // Включить режим измерения
  writeRegister(REG_DATA_FORMAT, 0x00); // Диапазон ±2g (по умолчанию)
}

void loop() {
  int16_t z_raw = readZAxis(); // Чтение сырых данных
  float z_mss = z_raw * scale_factor / 9.81; // Перевести в м/с²

  Serial.print("Z: ");
  Serial.print(z_mss);
  Serial.println(" м/с²");
  delay(100);
}

// Функция записи в регистр
void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(DEVICE_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Функция чтения данных по оси Z
int16_t readZAxis() {
  Wire.beginTransmission(DEVICE_ADDR);
  Wire.write(REG_DATAZ0);
  Wire.endTransmission(false);

  Wire.requestFrom(DEVICE_ADDR, 2); // Запросить 2 байта
  uint8_t z0 = Wire.read(); // Младший байт
  uint8_t z1 = Wire.read(); // Старший байт

  // Объединить байты с учетом знака
  return (int16_t)(z1 << 8 | z0);
}
