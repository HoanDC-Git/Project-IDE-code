#include <Arduino.h>

#define Sensor_Home_Pin 2           //X+
#define Sensor_Countshelf_Pin 3    //X-

void setup() {
  Serial.begin(9600);
  pinMode(Sensor_Home_Pin, INPUT);
  pinMode(Sensor_Countshelf_Pin, INPUT);
}

void loop() {
  int isHome  = digitalRead(Sensor_Home_Pin);
  int isShelf = digitalRead(Sensor_Countshelf_Pin);

  if (isHome == 0) {
    Serial.print("Home co vat       |      ");
  }
  else {
    Serial.print("Home ko vat       |      ");
  }
  if (isShelf == 0) {
    Serial.println("Shelf co vat");
  }
  else {
    Serial.println("Shelf ko vat");
  }

  delay(50);
}
