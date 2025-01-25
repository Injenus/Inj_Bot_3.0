/*
   0 - основание руки
   1 .. 3 - звенья руки
   4 - схват
   5 - основание камеры
   6 - высота камеры
*/
#include <ServoDriverSmooth.h>


const byte servoNum = 7;
const uint16_t addr = 42;

ServoDriverSmooth servo[servoNum](0x40);

struct ServoSettings {
  int min_mcs[servoNum] = {84, 116, 116, 116, 300, 135, 150};
  int max_mcs[servoNum] = {582, 542, 542, 542, 492, 525, 467};
  int min_angles[servoNum] = {0, 0, 0, 0, 0, 0, 0};
  int max_angles[servoNum] = {270, 270, 270, 270, 100, 180, 100};
};
ServoSettings servoSettings;

struct ServoPosMcs {
  //int pos[servoNum];
  int pos[servoNum] = {135, 135, 135, 135, 120, 135, 100};
};
ServoPosMcs servoPosMcs;

uint32_t timer = 0;


int anglesToMcs(int angles, byte idx) {
  int from_a_min = servoSettings.min_angles[idx];
  int from_a_max = servoSettings.max_angles[idx];
  int to_mcs_min = servoSettings.min_mcs[idx];
  int to_mcs_max = servoSettings.max_mcs[idx];
  return map(angles, from_a_min, from_a_max, to_mcs_min, to_mcs_max);
}

void setup() {
  Serial.begin(115200);
  //EEPROM.get(addr, servoPosMcs);

  servo[0].attach(0, servoSettings.min_mcs[0], servoSettings.max_mcs[0]);
  servo[0].writeMicroseconds(servoPosMcs.pos[0]);
  servo[0].setAutoDetach(false);
  servo[0].setSpeed(240);
  servo[0].setAccel(40);
  
  servo[1].attach(1, servoSettings.min_mcs[1], servoSettings.max_mcs[1]);
  servo[1].writeMicroseconds(servoPosMcs.pos[1]);
  servo[1].setAutoDetach(false);
  servo[1].setSpeed(240);
  servo[1].setAccel(40);

  servo[2].attach(2, servoSettings.min_mcs[2], servoSettings.max_mcs[2]);
  servo[2].writeMicroseconds(servoPosMcs.pos[2]);
  servo[2].setAutoDetach(false);
  servo[2].setSpeed(240);
  servo[2].setAccel(40);

  servo[3].attach(3, servoSettings.min_mcs[3], servoSettings.max_mcs[3]);
  servo[3].writeMicroseconds(servoPosMcs.pos[3]);
  servo[3].setAutoDetach(false);
  servo[3].setSpeed(240);
  servo[3].setAccel(40);

  servo[4].attach(4, servoSettings.min_mcs[4], servoSettings.max_mcs[4]);
  servo[4].writeMicroseconds(servoPosMcs.pos[4]);
  servo[4].setAutoDetach(false);
  servo[4].setSpeed(240);
  servo[4].setAccel(40);

  servo[5].attach(5, servoSettings.min_mcs[4], servoSettings.max_mcs[4]);
  servo[5].writeMicroseconds(servoPosMcs.pos[4]);
  servo[5].setAutoDetach(false);
  servo[5].setSpeed(360);
  servo[5].setAccel(360);

  servo[6].attach(6, servoSettings.min_mcs[5], servoSettings.max_mcs[5]);
  servo[6].writeMicroseconds(servoPosMcs.pos[5]);
  servo[6].setAutoDetach(false);
  servo[6].setSpeed(360);
  servo[6].setAccel(360);

  pinMode(14, INPUT);
}

void loop() {
  for (int i = 0; i++; i < servoNum) {
    servo[i].tick();
  }
  
  int potent =  analogRead(14);

  for (int i = 0; i < servoNum; i++) {
    servoPosMcs.pos[i] = map(potent, 0, 1023, servoSettings.min_mcs[servoNum], servoSettings.max_mcs[servoNum]);
    Serial.print("mcS ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(servoPosMcs.pos[i]);
    servo[i].writeMicroseconds(servoPosMcs.pos[i]);
  }
 



}
