// // //---------------------------------------------------------------// // //
// // ---------------- LIBRARIES ----------------- // // //
#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// // //---------------------------------------------------------------// // //
// // ---------------- DEFINE ----------------- // // //
// Động cơ Carousel (Trục X trên RAMPS)
#define Carousel_Motor_Step_Pin       54
#define Carousel_Motor_Dir_Pin        55
#define Carousel_Motor_En_Pin         38
#define Carousel_Motor_Home_Pin       2   // X+
#define Carousel_Motor_Countshelf_Pin 3   // X-

// Động cơ Gantry Y (Trục Y trên RAMPS)
#define Gantry_MotorY_Step_Pin        60
#define Gantry_MotorY_Dir_Pin         61
#define Gantry_MotorY_En_Pin          56
#define Gantry_MotorY_LimitSwitch_Pin 14  // Y-

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
#define Servo_Pin                     4   // 100 thu vao, 80 dua ra
#define Servo_LimitSwitch_Pin         19  // z+


// // //---------------------------------------------------------------// // //
// // AccelStepper(interface, stepPin, dirPin)
AccelStepper carouselMotor(AccelStepper::DRIVER, Carousel_Motor_Step_Pin, Carousel_Motor_Dir_Pin);
AccelStepper gantryMotorY(AccelStepper::DRIVER, Gantry_MotorY_Step_Pin, Gantry_MotorY_Dir_Pin);
AccelStepper gantryMotorZ1(AccelStepper::DRIVER, Gantry_MotorZ1_Step_Pin, Gantry_MotorZ1_Dir_Pin);
AccelStepper gantryMotorZ2(AccelStepper::DRIVER, Gantry_MotorZ2_Step_Pin, Gantry_MotorZ2_Dir_Pin);

Servo gantryServo;
// // //---------------------------------------------------------------// // //
// // ---------------- CONSTANTS ----------------- // //
// Carousel Motor
const int CAROUSEL_MOTOR_MICROSTEP        = 4;
const int CAROUSEL_MOTOR_MAXSPEED         = -100;
const int CAROUSEL_MOTOR_HOMESPEED        = CAROUSEL_MOTOR_MAXSPEED;
const int SHELF_NUMBER                    = 8;
const int DELAY_SAFE                      = 2000;

// Gantry Motor Y
const int GANTRY_MOTORY_MICROSTEP         = 4;
const int GANTRY_MOTORY_MAXSPEED          = 1000;
const int GANTRY_MOTORY_HOMESPEED         = - 300;

// Gantry Motor Z
const int GANTRY_MOTORZ_MICROSTEP         = 2;
const int GANTRY_MOTORZ_MAXSPEED          = 1000;
const int GANTRY_MOTORZ_HOMESPEED         = - 500;

const int ACCELERATION                    = 2000;
const int STEPS_PER_ROUND                 = 200;

const float GANTRY_Y_PULLEY_DIAMETER      = 20.0;
const float GANTRY_Y_PULLEY_CIRCUMFERENCE = GANTRY_Y_PULLEY_DIAMETER * 3.14159;

const float GANTRY_Z_PITCH                = 8.0;

const float PICKUP_Y                      = 560.0;
const float PICKUP_Z[]                    = {0.0, 50.0, 190.0};

const float DROPOFF_Y                     = 100.0;
const float DROPOFF_Z                     = 10.0;

const float Z_OOFSET                      = 20.0;


// // ---------------- VARIABLES ----------------- // //
int carousel_current_shelf          = 2;


// // //---------------------------------------------------------------// // //
// // ---------------- SETUP ----------------- // // //
void setup() {
  Serial.begin(115200);

  // --- Cấu hình động cơ Carousel ---
  carouselMotor.setEnablePin(Carousel_Motor_En_Pin);
  carouselMotor.setPinsInverted(false, false, true);
  carouselMotor.setMaxSpeed(CAROUSEL_MOTOR_MAXSPEED * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.setAcceleration(ACCELERATION * CAROUSEL_MOTOR_MICROSTEP);
  carouselMotor.enableOutputs();
  pinMode(Carousel_Motor_Home_Pin, INPUT);
  pinMode(Carousel_Motor_Countshelf_Pin, INPUT);

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
void homing(){
  // Servo logic
  gantryServo.write(100);
  while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
    delay(50);
  }
  gantryServo.write(90);

  bool carousel_homing_done = false;
  bool gantryY_homing_done  = false;
  bool gantryZ_homing_done  = false;

  carouselMotor.setSpeed(CAROUSEL_MOTOR_HOMESPEED * CAROUSEL_MOTOR_MICROSTEP);
  gantryMotorY.setSpeed(GANTRY_MOTORY_HOMESPEED * GANTRY_MOTORY_MICROSTEP);
  gantryMotorZ1.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);

  while (!carousel_homing_done || !gantryY_homing_done || !gantryZ_homing_done) {
    // Carousel logic
    if (digitalRead(Carousel_Motor_Home_Pin) == LOW) {
      carousel_homing_done = true;
      carouselMotor.stop();
    }
    if (!carousel_homing_done) {
      carouselMotor.runSpeed();
    }

    // Gantry Y logic
    if (digitalRead(Gantry_MotorY_LimitSwitch_Pin) == LOW) {
      gantryY_homing_done = true;
      gantryMotorY.stop();
    }
    if (!gantryY_homing_done) {
      gantryMotorY.runSpeed();
    }

    // Gantry Z logic
    if (digitalRead(Gantry_MotorZ_LimitSwitch_Pin) == LOW) {
      gantryZ_homing_done = true;
      gantryMotorZ1.stop();
      gantryMotorZ2.stop();
    }
    if (!gantryZ_homing_done) {
      gantryMotorZ1.runSpeed();
      gantryMotorZ2.runSpeed();
    }
  }
  carousel_current_shelf = 2;
  gantryMotorY.setCurrentPosition(0);
  gantryMotorZ1.setCurrentPosition(0);
  gantryMotorZ2.setCurrentPosition(0);
}


// // ---------------- moving ----------------- // // //
void moving(int carousel_target, long gantryY_target, long gantryZ_target, int servo_state, int servo_delay){
  bool carousel_moving_done     = false;
  bool gantryY_moving_done      = false;
  bool gantryZ_moving_done      = false;
  int carousel_spacing_counts   = (carousel_target - carousel_current_shelf + SHELF_NUMBER) % SHELF_NUMBER;
  int count                     = 0;
  unsigned long current_time    = millis();
  unsigned long last_debounce_time = 0;
  int current_sensor_state      = 0;
  int last_sensor_state         = 0;
  int min_debounce              = 500;

  long gantryY_steps            = gantryY_target / GANTRY_Y_PULLEY_CIRCUMFERENCE * STEPS_PER_ROUND * GANTRY_MOTORY_MICROSTEP;
  long gantryZ_steps            = gantryZ_target / GANTRY_Z_PITCH * STEPS_PER_ROUND * GANTRY_MOTORZ_MICROSTEP;

  // Set targetS
  carouselMotor.setSpeed(CAROUSEL_MOTOR_MAXSPEED * CAROUSEL_MOTOR_MICROSTEP);
  gantryMotorY.moveTo(gantryY_steps);
  gantryMotorZ1.moveTo(gantryZ_steps);
  gantryMotorZ2.moveTo(gantryZ_steps);

  while (!carousel_moving_done || !gantryY_moving_done || !gantryZ_moving_done) {
    // Carousel logic
    if (count >= carousel_spacing_counts) {
      carousel_moving_done = true;
      carouselMotor.stop();
      carousel_current_shelf = carousel_target;
    }

    if (!carousel_moving_done) {
      current_time = millis();
      current_sensor_state = digitalRead(Carousel_Motor_Countshelf_Pin);
      if (current_sensor_state != last_sensor_state) {
        if (current_sensor_state == 0 && (current_time - last_debounce_time) > min_debounce) {
          last_sensor_state = current_sensor_state;
          last_debounce_time = current_time;
          count ++;
          carousel_current_shelf ++;
          if (carousel_current_shelf > SHELF_NUMBER) {
            carousel_current_shelf = 1;
          }
          Serial.print("SHELF:");
          Serial.println(carousel_current_shelf);
        }
        else if (current_time - last_debounce_time > min_debounce) {
          last_sensor_state = current_sensor_state;
          last_debounce_time = current_time;
        }
      }
      carouselMotor.runSpeed();
    }

    // Gantry Y logic
    if (gantryMotorY.distanceToGo() == 0) {
      gantryY_moving_done = true;
      gantryMotorY.stop();
    }
    if (!gantryY_moving_done) {
      gantryMotorY.run();
    }

    // Gantry Z logic
    if (gantryMotorZ1.distanceToGo() == 0) {
      gantryZ_moving_done = true;
      gantryMotorZ1.stop();
      gantryMotorZ2.stop();
    }
    if (!gantryZ_moving_done) {
      gantryMotorZ1.run();
      gantryMotorZ2.run();
    }
  }
  // Servo logic
  delay(100);
  if (servo_state == 1 && digitalRead(Servo_LimitSwitch_Pin) == 0) {
    gantryServo.write(80);
    delay(servo_delay);
    gantryServo.write(90);
  }
  else if (servo_state == 0 && digitalRead(Servo_LimitSwitch_Pin) == 1) {
    gantryServo.write(100);
    unsigned long start_time = millis();
    while (digitalRead(Servo_LimitSwitch_Pin) == 1) {
      if (millis() - start_time > 5000) {
        break;
      }
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

    if (command.equalsIgnoreCase("HOMING")) {
      homing();
      Serial.println("DONE:2");
    }

    else if (command.startsWith("SET:")) {
      int colon_pos = command.indexOf(':');
      if (colon_pos != -1) {
        String valString = command.substring(colon_pos + 1);
        int val = valString.toInt();
        carousel_current_shelf = val;
        Serial.print("DONE:");
        Serial.println(carousel_current_shelf);
      }
    }

    else {
      int first_colon = command.indexOf(':');
      int second_colon = command.indexOf(':', first_colon + 1);

      if (first_colon == -1 || second_colon == -1) {
        Serial.println("Syntax error");
        return;
      }
      else {
        String param1_str = command.substring(first_colon + 1, second_colon);
        String param2_str = command.substring(second_colon + 1);

        int target_shelf = param1_str.toInt();
        int target_tray = param2_str.toInt();

        if (command.startsWith("FETCH:"))
        {
          while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
            gantryServo.write(100);
            delay(50);
          }
          gantryServo.write(90);

          moving(target_shelf, PICKUP_Y, PICKUP_Z[target_tray] - Z_OOFSET, 1, 1800);
          moving(target_shelf, PICKUP_Y, PICKUP_Z[target_tray] + Z_OOFSET, 0, 1800);
          moving(target_shelf, DROPOFF_Y, DROPOFF_Z, 1, 1800);
        }
        else if (command.startsWith("STORE:"))
        {
          while (digitalRead(Servo_LimitSwitch_Pin) == HIGH) {
            gantryServo.write(100);
            delay(50);
          }
          gantryServo.write(90);

          moving(target_shelf, PICKUP_Y, PICKUP_Z[target_tray] + Z_OOFSET, 1, 1900);
          moving(target_shelf, PICKUP_Y, PICKUP_Z[target_tray] - Z_OOFSET, 0, 1900);
        }
        Serial.print("DONE:");
        Serial.println(carousel_current_shelf);
      }
    }
  }
}
