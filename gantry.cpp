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
#define Gantry_MotorY_LimitSwitch_Pin 14  //Y-

// Động cơ Gantry Z1 (Trục Z trên RAMPS)
#define Gantry_MotorZ1_Step_Pin       46
#define Gantry_MotorZ1_Dir_Pin        48
#define Gantry_MotorZ1_En_Pin         62
// Động cơ Gantry Z2 (Trục E0 trên RAMPS)
#define Gantry_MotorZ2_Step_Pin       26
#define Gantry_MotorZ2_Dir_Pin        28
#define Gantry_MotorZ2_En_Pin         24
#define Gantry_MotorZ_LimitSwitch_Pin 18  // Z-

// Servo
#define Servo_Pin                     4
#define Servo_LimitSwitch_Pin         19  // Z+


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
const int STEPS_PER_ROUND                 = 200;

const float GANTRY_Y_PULLEY_DIAMETER      = 24.0;
const float GANTRY_Y_PULLEY_CIRCUMFERENCE = GANTRY_Y_PULLEY_DIAMETER * 3.14159;

const float GANTRY_Z_PITCH                = 8.0;

// Position
const float PICKUP_Y                      = 400.0;
const float PICKUP_Z[]                    = {0.0, 150.0, 250.0};
const float Z_OOFSET                      = 20.0;
const float DROPOFF_Y[]                   = {0.0, 100.0, 200.0};
const float DROPOFF_Z                     = 150.0;

// // ---------------- VARIABLES ----------------- // //

// // //---------------------------------------------------------------// // //
// // ---------------- SETUP ----------------- // // //
void setup() {
  Serial.begin(9600);
  Serial.println("HOMING hoac <pickup>:<dropof>; pickup:1-3, dropoff:1-2");
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

  gantryMotorY.moveTo(long(gantryY_target / GANTRY_Y_PULLEY_CIRCUMFERENCE * STEPS_PER_ROUND * GANTRY_MOTORY_MICROSTEP));
  gantryMotorZ1.moveTo(long(gantryZ_target / GANTRY_Z_PITCH * STEPS_PER_ROUND * GANTRY_MOTORZ_MICROSTEP));
  gantryMotorZ2.moveTo(long(gantryZ_target / GANTRY_Z_PITCH * STEPS_PER_ROUND * GANTRY_MOTORZ_MICROSTEP));

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
  delay(100);

  // Điều khiển servo
  if (servo_state == 1 && digitalRead(Servo_LimitSwitch_Pin) == LOW) {
    gantryServo.write(100);
    delay(1000);
    gantryServo.write(90);
  }
  else if (servo_state == 0 && digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
    while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
      gantryServo.write(80);
      delay(10);
    }
    gantryServo.write(90);
  }
  delay(100);
}


// // //---------------------------------------------------------------// // //
// // ---------------- MAIN FUNCTION ----------------- // // //
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("HOMING")) { homing(); }
    else {
      char command_buffer[50];

      // Copy chuỗi vào buffer (đảm bảo chuỗi command không quá 50 ký tự)
      command.toCharArray(command_buffer, 50);

      // --- Debug: In ra chuỗi buffer để kiểm tra xem nó nhận đúng không ---
      Serial.print("Buffer nhan duoc: ");
      Serial.println(command_buffer);

      // Tách chuỗi an toàn hơn
      char* token1 = strtok(command_buffer, ":");
      char* token2 = strtok(NULL, ":");

      if (token1 != NULL && token2 != NULL) {
        int pickup        = atoi(token1);
        int dropoff       = atoi(token2);
        if ( pickup >= 1 && pickup <=3 && dropoff >= 1 && dropoff <=2) {
          Serial.print("PICKUP 1-3: "); Serial.println(pickup);
          Serial.print("DROPOFF 1-2: "); Serial.println(dropoff);

          moving(PICKUP_Y, PICKUP_Z[pickup] - Z_OOFSET, 0);
          moving(PICKUP_Y, PICKUP_Z[pickup] + Z_OOFSET, 1);
          moving(PICKUP_Y, PICKUP_Z[pickup] + Z_OOFSET, 0);
          moving(DROPOFF_Y[dropoff], DROPOFF_Z + Z_OOFSET, 1);
          moving(DROPOFF_Y[dropoff], DROPOFF_Z - Z_OOFSET, 0);
        }
        else {
          Serial.println("<pickup>:<dropof>; pickup:1-3, dropoff:1-2");
        }
      }
      else {
        Serial.println("Loi: Sai dinh dang (VD: 1:2)");
      }
    }
  }
}
