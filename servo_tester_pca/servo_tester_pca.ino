/*
   Данный код плавно двигает туда-сюда одной сервой
   Используется драйвер PCA9685 
   Документация: https://alexgyver.ru/servosmooth/
*/

#include <ServoDriverSmooth.h>
//ServoDriverSmooth servo;
ServoDriverSmooth servo(0x40);      // с указанием адреса драйвера
//ServoDriverSmooth servo(0x40, 270); // с указанием адреса и макс. угла
ServoDriverSmooth servo_0(0x40);

uint32_t tmr;
boolean flag;

int min_m = 60;
int max_m = 640;

int mcs;

void setup() {
  Serial.begin(115200);
  //servo.start();
  Serial.println("setup");
  servo.attach(0, min_m, max_m);     // подключить
  servo_0.attach(1, min_m, max_m);
  //servo.smoothStart();
  servo.setAutoDetach(false);
  servo.setSpeed(90); // ограничить скорость
  servo.setAccel(1.0);   // установить ускорение (разгон и торможение)
  
  servo_0.setAutoDetach(false);
  servo_0.setSpeed(90); // ограничить скорость
  servo_0.setAccel(1.0);   // установить ускорение (разгон и торможение)

  Serial.println("end setup");
  pinMode(14, INPUT);
}
int scaling(int angle){
  int mcs = map(angle, 0, 270, min_m, max_m);
  Serial.print(angle);
  Serial.print(" _ ");
  Serial.println(mcs);
  return mcs;
}
void loop() {
  servo.tick();

//  if (millis() - tmr >= 4000) {   // каждые 3 сек
//    tmr = millis();
//    flag = !flag;
//    //servo.setTargetDeg(flag ? 0 : 90);
////    servo.setTarget(flag ? 50 : 350);
//    //servo.setTarget(flag ? 350 : 700);
//    servo.setTarget(flag ? 50 : 150);
  //}
//  static int mode = 0;
//  if (millis() - tmr >= 5000){
//    tmr = millis();
//    mode++;
//    if (mode > 6){
//      mode = 0;
//    }
//    switch(mode){
//      case 0:
//        servo.writeMicroseconds(scaling(45));
//      break;
//      case 1:
//        servo.writeMicroseconds(scaling(90));
//      break;
//      case 2:
//        servo.writeMicroseconds(scaling(135));
//      break;
//      case 3:
//        servo.writeMicroseconds(scaling(180));
//      break;
//      case 4:
//        servo.writeMicroseconds(scaling(225));
//      break;
//      case 5:
//        servo.writeMicroseconds(scaling(135));
//      break;
//      case 6:
//        servo.writeMicroseconds(scaling(135));
//      break;
//    }
//
//  }
    mcs = map(analogRead(14), 0, 1023, min_m, max_m);
    Serial.println(mcs);
    servo.writeMicroseconds(mcs);
   // servo_0.writeMicroseconds(mcs);




}
