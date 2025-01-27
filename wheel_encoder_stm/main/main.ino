#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

// UART настройки
#define UART_BAUD_RATE 115200

// Глобальные переменные
volatile long encPositions[4] = {0, 0, 0, 0}; // Позиции для 4 энкодеров
float rpms[4] = {0.0, 0.0, 0.0, 0.0};         // Скорости в RPM
unsigned long lastTime = 0;                   // Время последнего расчёта

// Последние состояния сигналов энкодеров
volatile bool lastStates[4][2] = {
  {0, 0}, {0, 0}, {0, 0}, {0, 0}
};

// UART
HardwareSerial serialPort(1);

// Функция отправки данных (работает на втором ядре)
void sendTask(void *param) {
  while (true) {
    // Создаём пакет
    uint8_t packet[19];
    packet[0] = 'S'; // Заголовок

    // Копируем скорости в пакет
    for (int i = 0; i < 4; i++) {
      float speed = rpms[i];
      memcpy(&packet[1 + i * 4], &speed, 4);
    }

    // Рассчитываем контрольную сумму Флетчер-16
    uint16_t checksum = 0xFFFF; // Инициализация
    for (int i = 0; i < 17; i++) {
      checksum = (checksum & 0xFF) + packet[i];
      checksum = (checksum >> 8) + (checksum & 0xFF);
    }
    checksum = (checksum >> 8) + (checksum & 0xFF);
    checksum &= 0xFF; // Окончательный результат
    checksum |= (checksum << 8); // 16-битная контрольная сумма

    // Добавляем контрольную сумму в пакет
    packet[17] = checksum & 0xFF;
    packet[18] = (checksum >> 8) & 0xFF;

    // Отправляем пакет
    serialPort.write(packet, sizeof(packet));

    // Задержка для заданной частоты (периода)
    vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL));
  }
}

void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  serialPort.begin(UART_BAUD_RATE, SERIAL_8N1, -1, -1); // UART1, TX/RX не указаны (по умолчанию)

  // Настройка пинов энкодеров
  setupEncoder(ENC1_A, ENC1_B, 0);
  setupEncoder(ENC2_A, ENC2_B, 1);
  setupEncoder(ENC3_A, ENC3_B, 2);
  setupEncoder(ENC4_A, ENC4_B, 3);

  // Запуск задачи отправки на втором ядре
  xTaskCreatePinnedToCore(sendTask, "SendTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  unsigned long currentTime = millis();

  // Обновляем скорости каждые UPDATE_INTERVAL мс
  if (currentTime - lastTime >= UPDATE_INTERVAL) {
    noInterrupts(); // Отключаем прерывания временно
    for (int i = 0; i < 4; i++) {
      long position = encPositions[i]; // Копируем положение
      encPositions[i] = 0;            // Сбрасываем положение
      rpms[i] = (position / float(STEPS_PER_REV)) * (60000.0 / UPDATE_INTERVAL);
    }
    interrupts(); // Включаем прерывания обратно
    lastTime = currentTime; // Обновляем время последнего расчёта
  }
}

// Настройка пинов энкодеров и прерываний
void setupEncoder(uint8_t pinA, uint8_t pinB, int index) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), [=]() { handleEncoder(pinA, pinB, index); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), [=]() { handleEncoder(pinA, pinB, index); }, CHANGE);
}

// Обработчик прерываний для энкодеров
void handleEncoder(uint8_t pinA, uint8_t pinB, int index) {
  bool currentA = digitalRead(pinA);
  bool currentB = digitalRead(pinB);

  if (lastStates[index][0] != currentA || lastStates[index][1] != currentB) {
    if (lastStates[index][0] == currentB) {
      encPositions[index]++;
    } else {
      encPositions[index]--;
    }
  }

  lastStates[index][0] = currentA;
  lastStates[index][1] = currentB;
}
