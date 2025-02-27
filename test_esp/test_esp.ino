// --- Назначение пинов энкодеров ---
#define ENC3_A 32
#define ENC3_B 33

#define ENC4_A 25
#define ENC4_B 26

#define ENC2_A 27
#define ENC2_B 14

#define ENC1_A 15  // заменили с 12 на 15 для безопасности
#define ENC1_B 2   // безопасный пин для второго канала

// --- Назначение ШИМ-пинов ---
#define PWM1_A 4
#define PWM1_B 16

#define PWM2_A 17
#define PWM2_B 18

#define PWM3_A 19
#define PWM3_B 21

#define PWM4_A 22
#define PWM4_B 23

#define PWM_DEF 5

#define PWM_FREQ 16000
#define PWM_RESOL 12  // 4096

volatile long encoderPositions[4] = {0, 0, 0, 0};

// --- Обработчики прерываний для энкодеров ---
void IRAM_ATTR handleEncoder1A() { encoderPositions[0] += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void IRAM_ATTR handleEncoder1B() { encoderPositions[0] += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? 1 : -1; }

void IRAM_ATTR handleEncoder2A() { encoderPositions[1] += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }
void IRAM_ATTR handleEncoder2B() { encoderPositions[1] += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? 1 : -1; }

void IRAM_ATTR handleEncoder3A() { encoderPositions[2] += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? 1 : -1; }
void IRAM_ATTR handleEncoder3B() { encoderPositions[2] += (digitalRead(ENC3_A) != digitalRead(ENC3_B)) ? 1 : -1; }

void IRAM_ATTR handleEncoder4A() { encoderPositions[3] += (digitalRead(ENC4_A) == digitalRead(ENC4_B)) ? 1 : -1; }
void IRAM_ATTR handleEncoder4B() { encoderPositions[3] += (digitalRead(ENC4_A) != digitalRead(ENC4_B)) ? 1 : -1; }

void setupEncoder(uint8_t pinA, uint8_t pinB, void (*isrA)(), void (*isrB)()) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
}

void setupPWM(uint8_t pin) {
    ledcAttach(pin, PWM_FREQ, PWM_RESOL);
    ledcWrite(pin, 0);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  // --- Настройка энкодеров ---
  setupEncoder(ENC1_A, ENC1_B, handleEncoder1A, handleEncoder1B);
  setupEncoder(ENC2_A, ENC2_B, handleEncoder2A, handleEncoder2B);
  setupEncoder(ENC3_A, ENC3_B, handleEncoder3A, handleEncoder3B);
  setupEncoder(ENC4_A, ENC4_B, handleEncoder4A, handleEncoder4B);

  // --- Настройка ШИМ (каналы с 0 по 7) ---
  setupPWM(PWM_DEF);
  setupPWM(PWM1_A);
  setupPWM(PWM1_B);
  setupPWM(PWM2_A);
  setupPWM(PWM2_B);
  setupPWM(PWM3_A);
  setupPWM(PWM3_B);
  setupPWM(PWM4_A);
  setupPWM(PWM4_B);

  ledcWrite(PWM1_A, 0);
  ledcWrite(PWM1_B, 256);

  ledcWrite(PWM2_A, 0);
  ledcWrite(PWM2_B, 768);

  ledcWrite(PWM3_A, 0);
  ledcWrite(PWM3_B, 1536);

  ledcWrite(PWM4_A, 0);
  ledcWrite(PWM4_B, 3328);

  Serial.println("Система с 4 энкодерами и 4 парами ШИМ-пинами запущена");
}

// Структура для конвертации float в байты
union FloatConverter {
  float value;
  uint8_t bytes[4];
};

// Вычисление контрольной суммы Флетчера-16
void fletcher16(uint8_t *data, size_t len, uint8_t *sum1, uint8_t *sum2) {
  *sum1 = 0;
  *sum2 = 0;
  for(size_t i = 0; i < len; ++i) {
      *sum1 += data[i];
      *sum2 += *sum1;
  }
}

void loop() {
  static unsigned long lastRPMTime = 0;
  static long prevPositions[4] = {0};
  unsigned long currentTime = millis();

  if (currentTime - lastRPMTime >= 100) {
      unsigned long interval = currentTime - lastRPMTime;
      lastRPMTime = currentTime;

      noInterrupts();
      long currentPositions[4] = {encoderPositions[0], encoderPositions[1], 
                                 encoderPositions[2], encoderPositions[3]};
      interrupts();

      // Расчет RPM
      float rpm[4];
      for (int i = 0; i < 4; i++) {
          long delta = currentPositions[i] - prevPositions[i];
          rpm[i] = (delta * 60000.0) / (234.3 * interval);
          prevPositions[i] = currentPositions[i];
      }

      // Подготовка буфера данных
      uint8_t buffer[17]; // 1(S) + 4*4(float) = 17 байт
      buffer[0] = 'S';    // Стартовый байт
      
      // Заполнение данных (порядок: левый передний, левый задний, правый передний, правый задний)
      FloatConverter conv;
      for(int i = 0; i < 4; i++) {
          conv.value = rpm[i];
          memcpy(&buffer[1 + i*4], conv.bytes, 4);
      }

      // Расчет контрольной суммы
      uint8_t sum1, sum2;
      fletcher16(buffer, sizeof(buffer), &sum1, &sum2);
      
      // Отправка данных (S + данные + сумма)
      Serial.write(buffer, sizeof(buffer));
      Serial.write(sum1);
      Serial.write(sum2);
  }
}