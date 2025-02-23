#include <HardwareSerial.h>

// Параметры энкодеров
#define STEPS_PER_REV 234.3   // Шагов на один оборот вала редуктора
#define UPDATE_INTERVAL 20    // Интервал обновления скорости (мс)

// Пины энкодеров
#define ENC1_A 2
#define ENC1_B 4
#define ENC2_A 16
#define ENC2_B 17
#define ENC3_A 18
#define ENC3_B 19
#define ENC4_A 23
#define ENC4_B 25

// Глобальные переменные
volatile long encPositions[4] = {0, 0, 0, 0}; // Позиции для 4 энкодеров
float rpms[4] = {0.0, 0.0, 0.0, 0.0};         // Скорости в RPM
unsigned long lastTime = 0;                   // Время последнего расчёта

// Последние состояния сигналов энкодеров
volatile bool lastStates[4][2] = {
  {0, 0}, {0, 0}, {0, 0}, {0, 0}
};


// Прототипы функций
void setupEncoder(uint8_t pinA, uint8_t pinB, int index);
void IRAM_ATTR handleEncoder1();
void IRAM_ATTR handleEncoder2();
void IRAM_ATTR handleEncoder3();
void IRAM_ATTR handleEncoder4();
void calculateSpeed(void *param);
void sendSpeedData(void *param);
void printSpeedsToSerial(void *param);
uint16_t calculateFletcher16(const uint8_t *data, size_t len);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 22, 21); // UART2 на пинах GPIO22 (TX), GPIO21 (RX)

  // Настройка энкодеров
  setupEncoder(ENC1_A, ENC1_B, 0);
  setupEncoder(ENC2_A, ENC2_B, 1);
  setupEncoder(ENC3_A, ENC3_B, 2);
  setupEncoder(ENC4_A, ENC4_B, 3);

  Serial.println("TEST___");

  // Задачи для FreeRTOS
  xTaskCreatePinnedToCore(calculateSpeed, "CalculateSpeed", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sendSpeedData, "SendSpeedData", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // Основной цикл пуст, так как вся логика вынесена в задачи
}

// Настройка пинов энкодеров и прерываний
void setupEncoder(uint8_t pinA, uint8_t pinB, int index) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  
  switch (index) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder1, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinB), handleEncoder1, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinB), handleEncoder2, CHANGE);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder3, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinB), handleEncoder3, CHANGE);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder4, CHANGE);
      attachInterrupt(digitalPinToInterrupt(pinB), handleEncoder4, CHANGE);
      break;
  }
}

// Обработчики прерываний
void IRAM_ATTR handleEncoder1() {
  bool currentA = digitalRead(ENC1_A);
  bool currentB = digitalRead(ENC1_B);

  if (lastStates[0][0] != currentA || lastStates[0][1] != currentB) {
    encPositions[0] += (lastStates[0][0] == currentB) ? 1 : -1;
  }

  lastStates[0][0] = currentA;
  lastStates[0][1] = currentB;
}

void IRAM_ATTR handleEncoder2() {
  bool currentA = digitalRead(ENC2_A);
  bool currentB = digitalRead(ENC2_B);

  if (lastStates[1][0] != currentA || lastStates[1][1] != currentB) {
    encPositions[1] += (lastStates[1][0] == currentB) ? 1 : -1;
  }

  lastStates[1][0] = currentA;
  lastStates[1][1] = currentB;
}

void IRAM_ATTR handleEncoder3() {
  bool currentA = digitalRead(ENC3_A);
  bool currentB = digitalRead(ENC3_B);

  if (lastStates[2][0] != currentA || lastStates[2][1] != currentB) {
    encPositions[2] += (lastStates[2][0] == currentB) ? 1 : -1;
  }

  lastStates[2][0] = currentA;
  lastStates[2][1] = currentB;
}

void IRAM_ATTR handleEncoder4() {
  bool currentA = digitalRead(ENC4_A);
  bool currentB = digitalRead(ENC4_B);

  if (lastStates[3][0] != currentA || lastStates[3][1] != currentB) {
    encPositions[3] += (lastStates[3][0] == currentB) ? 1 : -1;
  }

  lastStates[3][0] = currentA;
  lastStates[3][1] = currentB;
}

void calculateSpeed(void *param) {
  while (true) {
    unsigned long currentTime = millis();

    if (currentTime - lastTime >= UPDATE_INTERVAL) {
      noInterrupts();
      for (int i = 0; i < 4; i++) {
        long position = encPositions[i];
        encPositions[i] = 0;
        rpms[i] = (position / float(STEPS_PER_REV)) * (60000.0 / UPDATE_INTERVAL);
      }
      interrupts();

      lastTime = currentTime;
    }
    vTaskDelay(1);
  }
}

// Задача отправки данных
void sendSpeedData(void *param) {
  while (true) {
    uint8_t packet[19]; // Полный пакет данных
    packet[0] = 'S';    // Символ начала

    // Копируем скорости в пакет
    memcpy(&packet[1], &rpms[0], 4);
    memcpy(&packet[5], &rpms[1], 4);
    memcpy(&packet[9], &rpms[2], 4);
    memcpy(&packet[13], &rpms[3], 4);

    // Расчёт контрольной суммы
    uint16_t checksum = calculateFletcher16(packet, 17);
    packet[17] = checksum & 0xFF;
    packet[18] = (checksum >> 8) & 0xFF;

     Serial2.write(packet, 19);

    vTaskDelay(20);
  }
}

// ✅ Новая задача: вывод скоростей в Serial
void printSpeedsToSerial(void *param) {
  while (true) {
    noInterrupts();
    float speeds[4] = {rpms[0], rpms[1], rpms[2], rpms[3]};
    interrupts();

    for(byte i; i<4; i++){
      Serial.print(speeds[i]);
      Serial.print(',');
    }
    Serial.println('.');
//    Serial.printf("%.2f\t%.2f\t%.2f\t%.2f;\n", speeds[0], speeds[1], speeds[2], speeds[3]);
    vTaskDelay(20);  // Интервал обновления совпадает с вычислениями
  }
}

// Расчёт контрольной суммы Флетчер-16
uint16_t calculateFletcher16(const uint8_t *data, size_t len) {
  uint16_t sum1 = 0xFF, sum2 = 0xFF;

  while (len) {
    size_t tlen = (len > 20) ? 20 : len;
    len -= tlen;

    do {
      sum1 += *data++;
      sum2 += sum1;
    } while (--tlen);

    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
  }

  sum1 = (sum1 & 0xFF) + (sum1 >> 8);
  sum2 = (sum2 & 0xFF) + (sum2 >> 8);

  return (sum2 << 8) | sum1;
}
