// // //---------------------------------------------------------------// // //
// // ---------------- LIBRARIES ----------------- // // //
#include <Arduino.h>
#include <AccelStepper.h>

// // //---------------------------------------------------------------// // //
// // ---------------- DEFINE ----------------- // // //
// Động cơ Carousel (Trục X trên RAMPS)
#define Carousel_Motor_Step_Pin         54
#define Carousel_Motor_Dir_Pin          55
#define Carousel_Motor_En_Pin           38
#define Carousel_Motor_Home_Pin         2 // X+ Brown = VCC, Black = Signal, Blue = GND
#define Carousel_Motor_Countshelf_Pin   3 // X-   0 co vat, 1 khong co vat

// // //---------------------------------------------------------------// // //
// // AccelStepper(interface, stepPin, dirPin)
AccelStepper carouselMotor(AccelStepper::DRIVER, Carousel_Motor_Step_Pin, Carousel_Motor_Dir_Pin);

// // //---------------------------------------------------------------// // //
// // ---------------- CONSTANTS ----------------- // //
// Carousel Motor
const int CAROUSEL_MOTOR_MICROSTEP        = 4;
const int CAROUSEL_MOTOR_MAXSPEED         = -100;
const int CAROUSEL_MOTOR_HOMESPEED        = CAROUSEL_MOTOR_MAXSPEED;
const int SHELF_NUMBER                    = 8;

const int ACCELERATION                    = 2000;


// // ---------------- VARIABLES ----------------- // //
int carousel_current_position             = 10;

// // //---------------------------------------------------------------// // //
// // ---------------- SETUP ----------------- // // //
void setup() {
  Serial.begin(9600);
  Serial.println("San sang nhan lenh...");

  // --- Cấu hình động cơ Carousel ---
  carouselMotor.setEnablePin(Carousel_Motor_En_Pin);
  carouselMotor.setPinsInverted(false, false, true);
  carouselMotor.setMaxSpeed(CAROUSEL_MOTOR_MAXSPEED * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.setAcceleration(ACCELERATION * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.enableOutputs();
  pinMode(Carousel_Motor_Home_Pin, INPUT);
  pinMode(Carousel_Motor_Countshelf_Pin, INPUT);

}

void homing() {
  carouselMotor.setSpeed(CAROUSEL_MOTOR_HOMESPEED * CAROUSEL_MOTOR_MICROSTEP);
  while (digitalRead(Carousel_Motor_Home_Pin) == 1) {
    carouselMotor.runSpeed();
  }
  carouselMotor.stop();
  carousel_current_position = 2;
}

void move (int spacing) {
  if (spacing == 0) return;
  carouselMotor.setSpeed(CAROUSEL_MOTOR_MAXSPEED * CAROUSEL_MOTOR_MICROSTEP);
  int count = 0;
  int current_sensor_state = 0;
  int last_sensor_state = 0;
  unsigned long current_time = millis();
  unsigned long last_debounce_time = millis();
  int min_debounce = 500;

  while (count < spacing) {
    current_time = millis();
    current_sensor_state = digitalRead(Carousel_Motor_Countshelf_Pin);
    carouselMotor.runSpeed();

    if (current_sensor_state != last_sensor_state) {
      if (current_sensor_state == 0 && (current_time - last_debounce_time) > min_debounce) {
        last_sensor_state = current_sensor_state;
        last_debounce_time = current_time;
        count++;

        Serial.print("Da di qua:"); //Debug, nho cmt khi chay that ////////////////////////
        Serial.print(count);
        Serial.println(" ke");
      }
      else if ((current_time - last_debounce_time) > min_debounce) {
        last_sensor_state = current_sensor_state;
        last_debounce_time = current_time;
      }
    }
  }
  carouselMotor.stop();
}
// // //---------------------------------------------------------------// // //
// // ---------------- PROCEDURE ----------------- // //

// // //---------------------------------------------------------------// // //
// // ---------------- MAIN FUNCTION ----------------- // // //
void loop() {
  if (Serial.available() > 0) {
    int command = Serial.parseInt();

    // Xóa bộ đệm Serial khỏi các ký tự còn lại (ví dụ: '\r', '\n')
    while (Serial.available()) {
      Serial.read();
    }

    if (command == 1000) {
      homing();
    }
    else if (command != 0) {
      int spacing = (command - carousel_current_position + SHELF_NUMBER) % SHELF_NUMBER;
      move(spacing);
      carousel_current_position = command;
      Serial.print("DONE    ");
      Serial.println(carousel_current_position);
    }
  }
}
