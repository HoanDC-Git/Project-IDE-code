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
#define Carousel_Motor_LimitSwitch_Pin  3

// // //---------------------------------------------------------------// // //
// // AccelStepper(interface, stepPin, dirPin)
AccelStepper carouselMotor(AccelStepper::DRIVER, Carousel_Motor_Step_Pin, Carousel_Motor_Dir_Pin);

// // //---------------------------------------------------------------// // //
// // ---------------- CONSTANTS ----------------- // //
// Carousel Motor
const int CAROUSEL_MOTOR_MICROSTEP        = 4;
const int CAROUSEL_MOTOR_MAXSPEED         = 200;
const int CAROUSEL_MOTOR_HOMESPEED        = - CAROUSEL_MOTOR_MAXSPEED / 2;
const long MAX_CAROUSEL_POS               = 123445L; //##############################//
const float CAROUSEL_GEARBOX_RATIO        = 5.18;
const float CAROUSEL_WHEEL_DIAMETER       = 487.33;
const float CAROUSEL_WHEEL_CIRCUMFERENCE  = CAROUSEL_WHEEL_DIAMETER * 3.14159;
const float SHELF_SPACING                 = 254.044;

const int ACCELERATION                    = 2000;
const long INF                            = 31231025325L;
const int STEP_PER_ROUND                  = 200;


// // ---------------- VARIABLES ----------------- // //
int last_switch_state                     = HIGH;
unsigned long last_debounce_time          = 0;
const unsigned long debounce_delay        = 50;

// // //---------------------------------------------------------------// // //
// // ---------------- SETUP ----------------- // // //
void setup() {
  Serial.begin(115200);
  Serial.println("San sang nhan lenh...");

  // --- Cấu hình động cơ Carousel ---
  carouselMotor.setEnablePin(Carousel_Motor_En_Pin);
  carouselMotor.setPinsInverted(false, false, true);
  carouselMotor.setMaxSpeed(CAROUSEL_MOTOR_MAXSPEED * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.setAcceleration(ACCELERATION * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.enableOutputs();
  pinMode(Carousel_Motor_LimitSwitch_Pin, INPUT_PULLUP);
}

// // //---------------------------------------------------------------// // //
// // ---------------- PROCEDURE ----------------- // //

// // //---------------------------------------------------------------// // //
// // ---------------- MAIN FUNCTION ----------------- // // //
void loop() {
  if (Serial.available() > 0) {
    int command = Serial.read();
    if (command == 0) {
      carouselMotor.moveTo(-INF);
      Serial.println("Nhan cong tac de dung...");
      while (digitalRead(Carousel_Motor_LimitSwitch_Pin) == HIGH) { carouselMotor.run(); }
      carouselMotor.stop();
      while (carouselMotor.distanceToGo() != 0) { carouselMotor.run(); }
      carouselMotor.setCurrentPosition(0);
    }
    else if (command > 0) { carouselMotor.move(command * SHELF_SPACING / CAROUSEL_WHEEL_CIRCUMFERENCE * CAROUSEL_GEARBOX_RATIO * CAROUSEL_MOTOR_MICROSTEP * STEP_PER_ROUND); }
    else {                  carouselMotor.move(command * SHELF_SPACING / CAROUSEL_WHEEL_CIRCUMFERENCE * CAROUSEL_GEARBOX_RATIO * CAROUSEL_MOTOR_MICROSTEP * STEP_PER_ROUND); }
  }
  carouselMotor.run();
}
