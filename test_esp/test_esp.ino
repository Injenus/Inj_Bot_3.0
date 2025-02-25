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

void loop() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastPrintTime >= 10) {  // 100 Гц обновление
    lastPrintTime = currentTime;

    noInterrupts();
    long positions[4] = {encoderPositions[0], encoderPositions[1], encoderPositions[2], encoderPositions[3]};
    interrupts();

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "E1:%ld E2:%ld E3:%ld E4:%ld", positions[0], positions[1], positions[2], positions[3]);
    Serial.println(buffer);
    Serial2.println(buffer);
  }
}
