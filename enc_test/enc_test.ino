#define ENC_A 2          // Пин A энкодера
#define ENC_B 3          // Пин B энкодера
#define STEPS_PER_REV 234.3 // Количество шагов на один оборот вала редуктора (21.3 * 11)
#define UPDATE_INTERVAL 49 // Интервал расчёта скорости в мс (100 мс)

// Глобальные переменные
volatile long encPosition = 0;  // Положение энкодера
volatile bool lastA, lastB;     // Последние состояния пинов A и B
float rpm = 0;                  // Скорость в оборотах в минуту
unsigned long lastTime = 0;     // Время последнего расчёта

void setup() {
  Serial.begin(115200);

  // Настройка пинов энкодера
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Инициализация начальных состояний энкодера
  lastA = digitalRead(ENC_A);
  lastB = digitalRead(ENC_B);

  // Прерывания на оба пина
  attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), handleEncoder, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();

  // Обновляем скорость каждые UPDATE_INTERVAL мс
  if (currentTime - lastTime >= UPDATE_INTERVAL) {
    noInterrupts(); // Отключаем прерывания временно
    long enc_position = encPosition; // Копируем положение
    encPosition = 0;             // Сбрасываем положение для нового измерения
    interrupts(); // Включаем прерывания обратно

    // Рассчитываем RPM
    rpm = (enc_position / float(STEPS_PER_REV)) * (60000.0 / UPDATE_INTERVAL);

    // Выводим результат
    Serial.print("RPM: ");
    Serial.println(rpm);

    lastTime = currentTime; // Обновляем время последнего расчёта
  }
}

// Обработчик прерываний для энкодера
void handleEncoder() {
  bool currentA = digitalRead(ENC_A);
  bool currentB = digitalRead(ENC_B);

  // Определяем направление вращения
  if (lastA != currentA || lastB != currentB) {
    if (lastA == currentB) {
      encPosition++;
    } else {
      encPosition--;
    }
  }

  lastA = currentA;
  lastB = currentB;
}
