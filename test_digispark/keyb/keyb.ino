#include <digikeyboard.h>

void setup() {
  pinMode(1, OUTPUT);


  delay(100);
  digitalWrite(1, LOW);

  pinMode(4, OUTPUT);
  digitalWrite(1, HIGH);
  //DigiKeyboard.update();
  DigiKeyboard.delay(10000);
}

void loop() {
  DigiKeyboard.println("t");
  
  delay(200);
}
