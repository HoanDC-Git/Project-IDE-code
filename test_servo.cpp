#include <Arduino.h>
#include <Servo.h>

#define Servo_Pin 4
#define Servo_LimitSwitch_Pin 19  //  chan Z+ tren ramps

Servo myServo;

void setup() {
  Serial.begin(9600);
  pinMode(Servo_LimitSwitch_Pin, INPUT_PULLUP);
  myServo.attach(Servo_Pin);
  myServo.write(90);
  Serial.println("An 1 de thu vao");
  Serial.println("An 2 de dua ra");
  Serial.println("An 3 de thu vao, an cong tac hanh trinh de dung");
  Serial.println("An phim bat ky de dung lai");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '1') {
      myServo.write(100); // Quay nguoc chieu kim dong ho / thu vao
      delay(1000);
      myServo.write(90);
    } else if (input == '2') {
      myServo.write(80); // Quay cung chieu kim dong ho / dua ra
      delay(1000);
      myServo.write(90);
    } else if (input =='3'){
      myServo.write(100);
      while (true)
      {
        if (digitalRead(Servo_LimitSwitch_Pin) == 0) {break;}
      }
      myServo.write(90); // Dung lai
    }
  }
}
