#include <Arduino.h>

// Конфигурационные константы
#define RPM_MULTIPLIER 8192
#define ENCODER_STEPS_PER_REVOLUTION 2343 // Шагов энкодера на один оборот
#define ENCODER_STEPS_DIVIDER 10
#define UPDATE_INTERVAL_MS 4              // Интервал обновления в миллисекундах
#define NUMBER_OF_ENCODERS 4               // Количество энкодеров
#define DATA_PACKET_SIZE 19                // Размер пакета данных в байтах
#define PWM_FREQUENCY 16000                // Частота ШИМ в Гц
#define PWM_RESOLUTION_BITS 12             // Разрешение ШИМ (макс. значение 4095)
#define MAX_PWM_VALUE 4095                 // Максимальное значение ШИМ
#define RPM_DEADZONE 42                // Зона нечувствительности регулятора

// Пины подключения энкодеров [A, B]
const uint8_t ENCODER_PINS[4][2] = {
  {2, 15},  // Энкодер 1
  {27, 14}, // Энкодер 2
  {32, 33}, // Энкодер 3
  {26, 25}  // Энкодер 4
};

// Пины управления моторами [A, B]
const uint8_t MOTOR_PINS[4][2] = {
  {16, 4},  // Мотор 1
  {17, 18}, // Мотор 2
  {21, 19}, // Мотор 3
  {22, 23}  // Мотор 4
};

// Структура для хранения состояния энкодера
struct Encoder {
  volatile int32_t step_count;    // Текущая позиция в шагах
  volatile uint8_t previous_state; // Предыдущее состояние контактов
};
volatile Encoder encoder_states[4]; // Массив энкодеров

// Структура для управления мотором
struct Motor {
  uint8_t pwm_channel_a;    // Номер канала ШИМ для выхода A
  uint8_t pwm_channel_b;    // Номер канала ШИМ для выхода B
  int32_t current_pwm;        // Текущее значение ШИМ  | int???
  int32_t target_rpm;         // Целевая скорость вращения
  int32_t measured_rpm;       // Измеренная скорость вращения
  int32_t prev_rpm;           //предыдущая измеренная скорость (для реалзации фильтра)
};
Motor motor_controllers[4]; // Массив моторов

// Глобальные переменные
volatile int32_t target_rpms[4] = {0}; // Целевые скорости для моторов
uint8_t serial_buffer[DATA_PACKET_SIZE]; // Буфер для приема данных
uint8_t buffer_position = 0;           // Текущая позиция в буфере
bool receiving_packet = false;         // Флаг приема пакета
int32_t divider_p_gain = 8; // Коэффициент пропорционального регулятора

// Объявления функций
void handle_encoder_isr(uint8_t encoder_index);
void encoder1_isr() { handle_encoder_isr(0); }
void encoder2_isr() { handle_encoder_isr(1); }
void encoder3_isr() { handle_encoder_isr(2); }
void encoder4_isr() { handle_encoder_isr(3); }
void setup_encoders();
void calculate_motor_speeds();
void send_serial_data();
bool process_data_packet();
uint16_t calculate_checksum(uint8_t* data, uint8_t length);
void update_motor_power(uint8_t motor_index);
uint8_t get_real_motor_idx(uint8_t motor_index);

// Начальная настройка
void setup() {
  Serial.begin(115200); // Инициализация последовательного порта
  
  setup_encoders(); // Настройка энкодеров
  
  // Инициализация моторов
  motor_controllers[0].pwm_channel_a = MOTOR_PINS[0][0];
  motor_controllers[0].pwm_channel_b = MOTOR_PINS[0][1];
  
  motor_controllers[1].pwm_channel_a = MOTOR_PINS[1][0];
  motor_controllers[1].pwm_channel_b = MOTOR_PINS[1][1];
  
  motor_controllers[2].pwm_channel_a = MOTOR_PINS[2][0];
  motor_controllers[2].pwm_channel_b = MOTOR_PINS[2][1];
  
  motor_controllers[3].pwm_channel_a = MOTOR_PINS[3][0];
  motor_controllers[3].pwm_channel_b = MOTOR_PINS[3][1];
  
  for(uint8_t i=0; i<4; i++){
    ledcAttach(motor_controllers[i].pwm_channel_a, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcWrite(motor_controllers[i].pwm_channel_a, 0);
    
    ledcAttach(motor_controllers[i].pwm_channel_b, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcWrite(motor_controllers[i].pwm_channel_b, 0);

    motor_controllers[i].current_pwm = 0;
    motor_controllers[i].target_rpm = 0;
    motor_controllers[i].measured_rpm = 0;
  }
}

// Основной цикл программы
void loop() {
  static uint32_t last_update = 0;    // Время последнего обновления
  static uint32_t last_data_time = 0; // Время последнего приема данных

  // Прием данных с последовательного порта
  while(Serial.available() > 0){
    uint8_t _byte = Serial.read();
    last_data_time = millis();
    Serial.println(_byte);
    
    if(_byte == 'S' && !receiving_packet){
      buffer_position = 0;
      serial_buffer[buffer_position] = _byte;
      buffer_position++;
      receiving_packet = true;
    }
    else if(receiving_packet && buffer_position < DATA_PACKET_SIZE){
      serial_buffer[buffer_position] = _byte;
      buffer_position++;
      
      if(buffer_position >= DATA_PACKET_SIZE){
        receiving_packet = false;
        if(process_data_packet()){
          // Успешно принятый пакет
        }
      }
    }
  }

  // Периодическое обновление системы
  if(millis() - last_update >= UPDATE_INTERVAL_MS){
    calculate_motor_speeds(); // Расчет текущих скоростей
    
    // Обновление целевых скоростей
    noInterrupts();
    for(uint8_t i=0; i<4; i++){
      motor_controllers[i].target_rpm = target_rpms[i];
    }
    interrupts();
    
    // Регулировка мощности моторов
    for(uint8_t i=0; i<4; i++){
      update_motor_power(i);
    }
    //update_motor_power(3);
    
    send_serial_data(); // Отправка данных
    last_update = millis();
  }

  // Сброс приема при простое
  if(receiving_packet && (millis() - last_data_time > 50)){
    receiving_packet = false;
  }
}

// Обработчик прерываний для энкодера
void handle_encoder_isr(uint8_t encoder_index) {
  // Чтение текущего состояния контактов
  uint8_t current_state = digitalRead(ENCODER_PINS[encoder_index][0]) << 1;
  current_state |= digitalRead(ENCODER_PINS[encoder_index][1]);
  
  // Определение направления вращения
  uint8_t state_change = (encoder_states[encoder_index].previous_state << 2) | current_state;
  const int8_t direction_table[] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};
  encoder_states[encoder_index].step_count += direction_table[state_change];
  
  encoder_states[encoder_index].previous_state = current_state; // Сохранение состояния
}

// Настройка энкодеров и прерываний
void setup_encoders() {
  for(uint8_t i=0; i<4; i++){
    // Настройка пинов как входы с подтяжкой
    pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
    pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
    
    // Инициализация начального состояния
    encoder_states[i].previous_state = digitalRead(ENCODER_PINS[i][0]) << 1;
    encoder_states[i].previous_state |= digitalRead(ENCODER_PINS[i][1]);
    
    // Назначение обработчиков прерываний
    switch(i){
      case 0:
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][0]), encoder1_isr, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][1]), encoder1_isr, CHANGE);
        break;
      case 1:
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][0]), encoder2_isr, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][1]), encoder2_isr, CHANGE);
        break;
      case 2:
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][0]), encoder3_isr, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][1]), encoder3_isr, CHANGE);
        break;
      case 3:
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][0]), encoder4_isr, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[i][1]), encoder4_isr, CHANGE);
        break;
    }
  }
}

// Расчет текущей скорости вращения
void calculate_motor_speeds() {
  static int32_t previous_steps[4] = {0}; // Предыдущие показания
  
  noInterrupts();
  for(uint8_t i=0; i<4; i++){
    int32_t steps = encoder_states[i].step_count - previous_steps[i];
    previous_steps[i] = encoder_states[i].step_count;
    
    // Расчет RPM: (шаги/обороты) * (миллисекунды в минуте / интервал)
    int32_t new_value = (RPM_MULTIPLIER * ENCODER_STEPS_DIVIDER * steps / ENCODER_STEPS_PER_REVOLUTION) * 3000;
    static uint8_t K = 4;
    motor_controllers[i].measured_rpm  = (motor_controllers[i].prev_rpm * ( (1 << K) - 1 ) + new_value) >> K;
    motor_controllers[i].prev_rpm = motor_controllers[i].measured_rpm;
    
  }
  interrupts();
}

// Отправка данных через последовательный порт
void send_serial_data() {
  uint8_t output_packet[19] = {'S'}; // Заголовок пакета
  
  // Запись значений RPM в пакет
  for(uint8_t i=0; i<4; i++){
    int32_t rpm_value = motor_controllers[i].measured_rpm;

    // Преобразуем в little-endian байты
    output_packet[1 + i*4] = (rpm_value)       & 0xFF; // Младший байт
    output_packet[2 + i*4] = (rpm_value >> 8)  & 0xFF;
    output_packet[3 + i*4] = (rpm_value >> 16) & 0xFF;
    output_packet[4 + i*4] = (rpm_value >> 24) & 0xFF; // Старший байт
  }
  
  // Расчет контрольной суммы (для 17 байт: 1 заголовок + 16 данных)
  uint16_t checksum = calculate_checksum(output_packet, 17);
  output_packet[17] = checksum >> 8;   // Старший байт контрольной суммы
  output_packet[18] = checksum & 0xFF; // Младший байт;
  
  //Serial.write(output_packet, 19); // Отправка пакета
  for(uint8_t i=0; i<4; i++) {
    int32_t rpm = *((int32_t*)&output_packet[1 + i*4]); // Для little-endian
    Serial.print("RPM ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print((float)rpm/RPM_MULTIPLIER);
    Serial.print("   ");
  }
  Serial.println();

}

// Обработка входящего пакета
bool process_data_packet() {
  // Проверка контрольной суммы
  uint16_t received_checksum = (serial_buffer[17] << 8) | serial_buffer[18];
  if(calculate_checksum(serial_buffer, 17) != received_checksum){
    return false;
  }

  // Чтение значений RPM
  noInterrupts();
  for(uint8_t i = 0; i < 4; i++) {
    // Создаем НЕ-volatile переменную для промежуточного копирования
    int32_t tmp;
    // Копируем 4 байта из serial_buffer (с учетом смещения, если нужно)
    memcpy(&tmp, &serial_buffer[1 + i * 4], sizeof(int32_t));
    // Присваиваем значение в volatile-массив
    target_rpms[i] = RPM_MULTIPLIER * tmp; // умнижаем рпм на местный коэффицент вычислений
  }
  interrupts();
  
  return true;
}

// Расчет контрольной суммы
uint16_t calculate_checksum(uint8_t* data, uint8_t length) {
  uint16_t sum1 = 0xFF;
  uint16_t sum2 = 0xFF;
  
  for(uint8_t i=0; i<length; i++){
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

// Обновление мощности мотора
void update_motor_power(uint8_t motor_index) {

  int32_t error = motor_controllers[motor_index].target_rpm - 
                motor_controllers[motor_index].measured_rpm;

  // Пропорциональное регулирование
  int32_t pwm_change = error / (RPM_MULTIPLIER * divider_p_gain);
  pwm_change = constrain(pwm_change, -420, 420);
  
  // Обновление значения PWM
  motor_controllers[motor_index].current_pwm += pwm_change;
  motor_controllers[motor_index].current_pwm = constrain(
    motor_controllers[motor_index].current_pwm,
    -MAX_PWM_VALUE,
    MAX_PWM_VALUE
  );

  //Serial.println(motor_controllers[motor_index].current_pwm);
  
  // Управление выходами
  uint32_t pwm_value = abs(motor_controllers[motor_index].current_pwm);

  if(motor_controllers[motor_index].current_pwm < -RPM_DEADZONE){
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_a, 0);
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_b, pwm_value);
  }
  else if(motor_controllers[motor_index].current_pwm > RPM_DEADZONE){
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_a, pwm_value);
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_b, 0);
  }
  else{
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_a, 0);
    ledcWrite(motor_controllers[get_real_motor_idx(motor_index)].pwm_channel_b, 0);
  }
}

uint8_t get_real_motor_idx(uint8_t motor_index) {
  switch (motor_index){
    case 0:
      motor_index = 3;
      break;
    case 1:
      motor_index = 2;
      break;
    case 2:
      motor_index = 0;
      break;
    case 3:
      motor_index = 1;
      break;
  }
  return motor_index;
}
