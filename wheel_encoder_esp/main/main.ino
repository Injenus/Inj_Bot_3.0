#include <Arduino.h>

// Конфигурация энкодеров
#define STEPS_PER_REV     234.3f    // Шагов на оборот вала
#define UPDATE_INTERVAL   20        // Интервал обновления (мс)
#define NUM_ENCODERS      4         // Количество энкодеров
#define PACKET_SIZE       19        // Размер пакета данных
#define PWM_FREQ          16000     // Частота ШИМ
#define PWM_RESOL         12        // Разрешение ШИМ (4096)
#define MAX_PWM           4095      // Максимальное значение ШИМ
#define DEADBAND_RPM      10.0f     // Мёртвая зона регулирования

// Пины энкодеров (A, B)
const uint8_t encoderPins[NUM_ENCODERS][2] = {
  {15, 2},   // Encoder 1
  {27, 14}, // Encoder 2
  {32, 33}, // Encoder 3
  {25, 26}  // Encoder 4
};

// Структура для хранения состояния
struct EncoderState {
  volatile int32_t position;
  volatile uint8_t lastState;
  float rpm;
};
volatile EncoderState encoders[NUM_ENCODERS] = {0};

// Целевые скорости для колёс
volatile float targetSpeeds[4] = {0}; // [FL, BL, FR, BR]

// Приёмный буфер
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
bool packetStarted = false;

// Назначение ШИМ-пинов [A, B]
const uint8_t motorPins[4][2] = {
  {4, 16},   // Motor 1 (FL)
  {17, 18},  // Motor 2 (BL)
  {19, 21},  // Motor 3 (FR)
  {22, 23}   // Motor 4 (BR)
};

// Структура для управления мотором
struct MotorController {
  uint8_t pinA;
  uint8_t pinB;
  float currentPWM;
  float targetRPM;
  float currentRPM;
};

MotorController motors[4];
float kP = 0.5f; // Коэффициент пропорционального регулятора

void IRAM_ATTR handleEncoderISR(uint8_t index);
void setupEncoders();
void calculateSpeed();
void sendSpeedData();
bool parseSpeedPacket();
uint16_t fletcher16(const uint8_t* data, size_t len);
void setupPWM();
void updateMotorPWM(uint8_t motorIndex);




void setup() {
  Serial.begin(115200);
  setupEncoders();
  setupPWM();
  
  // Инициализация моторов
  for(int i = 0; i < 4; i++) {
    motors[i].pinA = motorPins[i][0];
    motors[i].pinB = motorPins[i][1];
    motors[i].currentPWM = 0;
    motors[i].targetRPM = 0;
    motors[i].currentRPM = 0;
  }
}

void loop() {
  static uint32_t lastUpdate = 0;
  
  // Обработка входящих данных
  while(Serial.available()) {
    uint8_t c = Serial.read();
    
    if(c == 'S' && !packetStarted) {
      rxIndex = 0;
      rxBuffer[rxIndex++] = c;
      packetStarted = true;
    } 
    else if(packetStarted) {
      rxBuffer[rxIndex++] = c;
      
      if(rxIndex >= PACKET_SIZE) {
        packetStarted = false;
        if(parseSpeedPacket()) {
          // Успешно распаршен пакет
          // targetSpeeds содержит новые значения
        }
      }
    }
  }

  // Отправка данных и расчёт
  if(millis() - lastUpdate >= UPDATE_INTERVAL) {
    calculateSpeed(); // Обновляем motors[].currentRPM
    
    // Обновляем целевые скорости из парсера
    noInterrupts();
    for(int i = 0; i < 4; i++) {
      motors[i].targetRPM = targetSpeeds[i];
    }
    interrupts();
    
    // Регулировка всех моторов
    for(int i = 0; i < 4; i++) {
      updateMotorPWM(i);
    }

    sendSpeedData();
    lastUpdate = millis();
  }
}




// Парсинг пакета с проверкой контрольной суммы
bool parseSpeedPacket() {
  // Проверка размера
  if(rxIndex != PACKET_SIZE) return false;

  // Проверка контрольной суммы
  uint16_t receivedChecksum = (rxBuffer[PACKET_SIZE-1] << 8) | rxBuffer[PACKET_SIZE-2];
  uint16_t calculatedChecksum = fletcher16(rxBuffer, PACKET_SIZE-2);
  
  if(receivedChecksum != calculatedChecksum) return false;

  // Извлечение значений скоростей
  const uint8_t* data = rxBuffer + 1;
  volatile float* speeds[] = {&targetSpeeds[0], &targetSpeeds[1], 
                             &targetSpeeds[2], &targetSpeeds[3]};
  
  for(int i = 0; i < 4; i++) {
    float value;
    memcpy(&value, data + i*4, 4);
    *speeds[i] = value;
  }

  return true;
}

// Расчёт контрольной суммы
uint16_t fletcher16(const uint8_t* data, size_t len) {
  uint16_t sum1 = 0xFF, sum2 = 0xFF;
  while(len) {
    size_t tlen = len > 20 ? 20 : len;
    len -= tlen;
    do {
      sum1 += *data++;
      sum2 += sum1;
    } while(--tlen);
    sum1 = (sum1 & 0xFF) + (sum1 >> 8);
    sum2 = (sum2 & 0xFF) + (sum2 >> 8);
  }
  sum1 = (sum1 & 0xFF) + (sum1 >> 8);
  sum2 = (sum2 & 0xFF) + (sum2 >> 8);
  return (sum2 << 8) | sum1;
}

// Настройка энкодеров и прерываний
void setupEncoders() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    pinMode(encoderPins[i][0], INPUT_PULLUP);
    pinMode(encoderPins[i][1], INPUT_PULLUP);
    
    // Инициализация начального состояния
    encoders[i].lastState = (digitalRead(encoderPins[i][0]) << 1 | digitalRead(encoderPins[i][1]));
    
    // Привязка прерываний
    attachInterrupt(digitalPinToInterrupt(encoderPins[i][0]), []{ handleEncoderISR(i); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPins[i][1]), []{ handleEncoderISR(i); }, CHANGE);
  }
}

// Обработчик прерываний для всех энкодеров
void IRAM_ATTR handleEncoderISR(uint8_t index) {
  const uint8_t state = (digitalRead(encoderPins[index][0]) << 1) | digitalRead(encoderPins[index][1]);
  const uint8_t prev = encoders[index].lastState;
  encoders[index].lastState = state;

  // Таблица переходов для определения направления
  const int8_t transitions[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  encoders[index].position += transitions[(prev << 2) | state];
}

// Расчёт скорости вращения
// Функция calculateSpeed() должна обновлять motors[].currentRPM
void calculateSpeed() {
  static int32_t lastPositions[NUM_ENCODERS] = {0};
  noInterrupts();
  for(uint8_t i = 0; i < NUM_ENCODERS; i++) {
    const int32_t delta = encoders[i].position - lastPositions[i];
    lastPositions[i] = encoders[i].position;
    motors[i].currentRPM = (delta / STEPS_PER_REV) * (60000.0f / UPDATE_INTERVAL);
  }
  interrupts();
}

// Отправка данных по UART
void sendSpeedData() {
  uint8_t packet[1 + NUM_ENCODERS * 4 + 2] = {'S'};
  float* rpmData = (float*)(packet + 1);
  
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    rpmData[i] = encoders[i].rpm;
  }

  // Расчёт контрольной суммы Fletcher16
  uint16_t sum1 = 0xFF, sum2 = 0xFF;
  for (size_t i = 0; i < sizeof(packet)-2; i++) {
    sum1 = (sum1 + packet[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  packet[sizeof(packet)-2] = sum1;
  packet[sizeof(packet)-1] = sum2;

  Serial.write(packet, sizeof(packet));
}

void setupPWM() {
  for(int i = 0; i < 4; i++) {
    ledcSetup(i*2, PWM_FREQ, PWM_RESOL);    // Каналы 0,2,4,6 для пинов A
    ledcSetup(i*2+1, PWM_FREQ, PWM_RESOL); // Каналы 1,3,5,7 для пинов B
    ledcAttachPin(motorPins[i][0], i*2);
    ledcAttachPin(motorPins[i][1], i*2+1);
  }
}

void updateMotorPWM(uint8_t motorIndex) {
  MotorController* mc = &motors[motorIndex];
  float error = mc->targetRPM - mc->currentRPM;
  
  // Пропорциональное регулирование
  float pwmDelta = kP * error;
  
  // Ограничение скорости изменения
  pwmDelta = constrain(pwmDelta, -MAX_PWM/10.0f, MAX_PWM/10.0f);
  
  mc->currentPWM += pwmDelta;
  mc->currentPWM = constrain(mc->currentPWM, -MAX_PWM, MAX_PWM);

  // Управление направлениями
  if(fabs(error) > DEADBAND_RPM) {
    if(mc->currentPWM > 0) {
      ledcWrite(motorIndex*2, 0);
      ledcWrite(motorIndex*2+1, (uint32_t)fabs(mc->currentPWM));
    } else {
      ledcWrite(motorIndex*2+1, 0);
      ledcWrite(motorIndex*2, (uint32_t)fabs(mc->currentPWM));
    }
  } else {
    ledcWrite(motorIndex*2, 0);
    ledcWrite(motorIndex*2+1, 0);
  }
}