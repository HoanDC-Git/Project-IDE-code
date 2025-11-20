// // //---------------------------------------------------------------// // //
// // ---------------- LIBRARIES ----------------- // // //
#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// // //---------------------------------------------------------------// // //
// // ---------------- DEFINE ----------------- // // //

// Động cơ Gantry Y (Trục Y trên RAMPS)
#define Gantry_MotorY_Step_Pin        60
#define Gantry_MotorY_Dir_Pin         61
#define Gantry_MotorY_En_Pin          56
#define Gantry_MotorY_LimitSwitch_Pin 14

// Động cơ Gantry Z1 (Trục Z trên RAMPS)
#define Gantry_MotorZ1_Step_Pin       46
#define Gantry_MotorZ1_Dir_Pin        48
#define Gantry_MotorZ1_En_Pin         62
// Động cơ Gantry Z2 (Trục E0 trên RAMPS)
#define Gantry_MotorZ2_Step_Pin       26
#define Gantry_MotorZ2_Dir_Pin        28
#define Gantry_MotorZ2_En_Pin         24
#define Gantry_MotorZ_LimitSwitch_Pin 18

// Servo
#define Servo_Pin                     4
#define Servo_LimitSwitch_Pin         15 //##########################//


// // //---------------------------------------------------------------// // //
// // AccelStepper(interface, stepPin, dirPin)
AccelStepper gantryMotorY(AccelStepper::DRIVER, Gantry_MotorY_Step_Pin, Gantry_MotorY_Dir_Pin);
AccelStepper gantryMotorZ1(AccelStepper::DRIVER, Gantry_MotorZ1_Step_Pin, Gantry_MotorZ1_Dir_Pin);
AccelStepper gantryMotorZ2(AccelStepper::DRIVER, Gantry_MotorZ2_Step_Pin, Gantry_MotorZ2_Dir_Pin);

Servo gantryServo;
// // //---------------------------------------------------------------// // //
// // ---------------- CONSTANTS ----------------- // //

// Gantry Motor Y
const int GANTRY_MOTORY_MICROSTEP         = 16;
const int GANTRY_MOTORY_MAXSPEED          = 500;
const int GANTRY_MOTORY_HOMESPEED         = - GANTRY_MOTORY_MAXSPEED / 2;

// Gantry Motor Z
const int GANTRY_MOTORZ_MICROSTEP         = 4;
const int GANTRY_MOTORZ_MAXSPEED          = 1000;
const int GANTRY_MOTORZ_HOMESPEED         = - GANTRY_MOTORZ_MAXSPEED / 2;

const int ACCELERATION                    = 2000;
const long INF                            = 31231025325L;
const int STEPS_PER_ROUND                 = 200;

const float GANTRY_Y_PULLEY_DIAMETER      = 24.0;
const float GANTRY_Y_PULLEY_CIRCUMFERENCE = GANTRY_Y_PULLEY_DIAMETER * 3.14159;

const float GANTRY_Z_PITCH                = 8.0;

// // ---------------- VARIABLES ----------------- // //

// // //---------------------------------------------------------------// // //
// // ---------------- SETUP ----------------- // // //
void setup() {
  Serial.begin(115200);
  Serial.println("San sang nhan lenh...");

  // --- Cấu hình động cơ Gantry Y ---
  gantryMotorY.setEnablePin(Gantry_MotorY_En_Pin);
  gantryMotorY.setPinsInverted(false, false, true);
  gantryMotorY.setMaxSpeed(GANTRY_MOTORY_MAXSPEED * GANTRY_MOTORY_MICROSTEP);
  gantryMotorY.setAcceleration(ACCELERATION * GANTRY_MOTORY_MICROSTEP);
  gantryMotorY.enableOutputs();
  pinMode(Gantry_MotorY_LimitSwitch_Pin, INPUT_PULLUP);

  // --- Cấu hình động cơ Gantry Z1 ---
  gantryMotorZ1.setEnablePin(Gantry_MotorZ1_En_Pin);
  gantryMotorZ1.setPinsInverted(false, false, true);
  gantryMotorZ1.setMaxSpeed(GANTRY_MOTORZ_MAXSPEED * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ1.setAcceleration(ACCELERATION * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ1.enableOutputs();

  // --- Cấu hình động cơ Gantry Z2 ---
  gantryMotorZ2.setEnablePin(Gantry_MotorZ2_En_Pin);
  gantryMotorZ2.setPinsInverted(false, false, true);
  gantryMotorZ2.setMaxSpeed(GANTRY_MOTORZ_MAXSPEED * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.setAcceleration(ACCELERATION * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.enableOutputs();
  pinMode(Gantry_MotorZ_LimitSwitch_Pin, INPUT_PULLUP);

  gantryServo.attach(Servo_Pin);
  gantryServo.write(90);
  pinMode(Servo_LimitSwitch_Pin, INPUT_PULLUP);

  homing();
}


// // //---------------------------------------------------------------// // //
// // ---------------- PROCEDURE ----------------- // //
// // ---------------- homing ----------------- // // //
void homing() {
  bool gantryY_homed = false;
  bool gantryZ_homed = false;

  gantryMotorY.setSpeed(GANTRY_MOTORY_HOMESPEED * GANTRY_MOTORY_MICROSTEP);
  gantryMotorZ1.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);

  while (!gantryY_homed || !gantryZ_homed) {

    if (!gantryY_homed) {
      if (digitalRead(Gantry_MotorY_LimitSwitch_Pin) == LOW) {
        gantryY_homed = true;
        gantryMotorY.stop();
        gantryMotorY.setCurrentPosition(0);
      }
      else { gantryMotorY.runSpeed(); }
    }

    if (!gantryZ_homed) {
      if (digitalRead(Gantry_MotorZ_LimitSwitch_Pin) == LOW) {
        gantryZ_homed = true;
        gantryMotorZ1.stop();
        gantryMotorZ2.stop();
        gantryMotorZ1.setCurrentPosition(0);
        gantryMotorZ2.setCurrentPosition(0);
      }
      else {
        gantryMotorZ1.runSpeed();
        gantryMotorZ2.runSpeed();
      }
    }
  }
  while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
    gantryServo.write(80);
    delay(10);
  }
  gantryServo.write(90);
}


// // ---------------- moving ----------------- // // //
void moving(long gantryY_target, long gantryZ_target, int servo_state) {

  gantryMotorY.moveTo(gantryY_target / GANTRY_Y_PULLEY_CIRCUMFERENCE * STEPS_PER_ROUND * GANTRY_MOTORY_MICROSTEP);
  gantryMotorZ1.moveTo(gantryZ_target / GANTRY_Z_PITCH * STEPS_PER_ROUND * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.moveTo(gantryZ_target / GANTRY_Z_PITCH * STEPS_PER_ROUND * GANTRY_MOTORZ_MICROSTEP);

  // Bắt đầu di chuyển các động cơ
  while (gantryMotorY.distanceToGo() != 0 || gantryMotorZ1.distanceToGo() != 0) {
    gantryMotorY.run();
    gantryMotorZ1.run();
    gantryMotorZ2.run();
  }

  // Dừng tất cả các động cơ
  gantryMotorY.stop();
  gantryMotorZ1.stop();
  gantryMotorZ2.stop();

  // Điều khiển servo
  if (servo_state == 1 && digitalRead(Servo_LimitSwitch_Pin) == LOW) {
    gantryServo.write(100);
    delay(500);
    gantryServo.write(90);
  }
  else if (servo_state == 0 && digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
    while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
      gantryServo.write(80);
      delay(10);
    }
    gantryServo.write(90);
  }
}


// // //---------------------------------------------------------------// // //
// // ---------------- MAIN FUNCTION ----------------- // // //
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("HOMING")) { homing(); }
    else {
      char command_buffer[command.length() + 1];
      command.toCharArray(command_buffer, sizeof(command_buffer));

      long gantryY_target = atol(strtok(command_buffer, ":"));
      long gantryZ_target = atol(strtok(NULL, ":"));
      int servo_state     = atoi(strtok(NULL, ":"));
      moving(gantryY_target, gantryZ_target, servo_state);
    }
    Serial.println("DONE:");
  }
}
