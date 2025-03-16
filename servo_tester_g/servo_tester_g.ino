#include "ServoSmooth.h"
ServoSmooth servo;

uint32_t myTimer;

const int minWidth = 450;
const int maxWidth = 2550;

void setup() {
  Serial.begin(115200);
  servo.attach(10, minWidth, maxWidth);

  servo.setSpeed(45);   // ограничить скорость
  servo.setAccel(10.0);  // установить ускорение (разгон и торможение)
}

void loop() {
  // желаемая позиция задаётся методом setTa32232rget (импульс) или setTargetDeg (угол), далее
  // при вызове tick() производится автоматическое движение сервы
  // с заданным ускорением и ограничением скорости
  boolean state = servo.tick();   // здесь происходит движение серво по встроенному таймеру!


  if (millis() - myTimer >= 40) {
    myTimer = millis();
    int newPos = map(analogRead(A6), 0, 1023, minWidth, maxWidth); // берём с потенцометра значение 500-2400 (импульс)
    servo.setTarget(2000);               // и отправляем на серво
    Serial.println(String(newPos) + " " + String(servo.getCurrent())/* + " " + String(state)*/);
  // state показывает сотояние сервы (0 - движется, 1 - приехали и отключились)
  }
}
