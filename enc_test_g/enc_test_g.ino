#define ENC_A 2       // пин энкодера
#define ENC_B 3       // пин энкодера
volatile int encCounter;
volatile boolean flag, resetFlag;
volatile byte curState, prevState;
void setup() {
  Serial.begin(115200);
  attachInterrupt(0, int0, CHANGE);
  attachInterrupt(1, int0, CHANGE);
}
void int0() {
  encTick();
}

// алгоритм со сбросом от Ярослава Куруса
void encTick() {
  curState = digitalRead(ENC_A) | digitalRead(ENC_B) << 1;  // digitalRead хорошо бы заменить чем-нибудь более быстрым
  if (resetFlag && curState == 0b11) {
    if (prevState == 0b10) encCounter++;
    if (prevState == 0b01) encCounter--;
    resetFlag = 0;
    flag = true;
  }
  if (curState == 0b00) resetFlag = 1;
  prevState = curState;
}
void loop() {
  if (flag) {
    Serial.println(encCounter);
    flag = 0;
  }
}
