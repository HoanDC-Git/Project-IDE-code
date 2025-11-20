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
#define Carousel_Motor_LimitSwitch_Pin 3

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
AccelStepper carouselMotor(AccelStepper::DRIVER, Carousel_Motor_Step_Pin, Carousel_Motor_Dir_Pin);
AccelStepper gantryMotorY(AccelStepper::DRIVER, Gantry_MotorY_Step_Pin, Gantry_MotorY_Dir_Pin);
AccelStepper gantryMotorZ1(AccelStepper::DRIVER, Gantry_MotorZ1_Step_Pin, Gantry_MotorZ1_Dir_Pin);
AccelStepper gantryMotorZ2(AccelStepper::DRIVER, Gantry_MotorZ2_Step_Pin, Gantry_MotorZ2_Dir_Pin);

Servo gantryServo;
// // //---------------------------------------------------------------// // //
// // ---------------- CONSTANTS ----------------- // //
// Carousel Motor
const int CAROUSEL_MOTOR_MICROSTEP  = 4;
const int CAROUSEL_MOTOR_MAXSPEED   = 200;
const int CAROUSEL_MOTOR_HOMESPEED  = - CAROUSEL_MOTOR_MAXSPEED / 2;
const long MAX_CAROUSEL_POS         = 123445L; //##############################//

// Gantry Motor Y
const int GANTRY_MOTORY_MICROSTEP   = 16;
const int GANTRY_MOTORY_MAXSPEED    = 500;
const int GANTRY_MOTORY_HOMESPEED   = - GANTRY_MOTORY_MAXSPEED / 2;

// Gantry Motor Z
const int GANTRY_MOTORZ_MICROSTEP   = 4;
const int GANTRY_MOTORZ_MAXSPEED    = 1000;
const int GANTRY_MOTORZ_HOMESPEED   = - GANTRY_MOTORZ_MAXSPEED / 2;

const int ACCELERATION              = 2000;
const long INF                      = 31231025325L;


// // ---------------- VARIABLES ----------------- // //
int last_switch_state = HIGH;
unsigned long last_debounce_time = 0;
const unsigned long debounce_delay = 50;


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
void homing(){
  bool carousel_homed = false;
  bool gantryY_homed = false;
  bool gantryZ_homed = false;

  carouselMotor.setSpeed(CAROUSEL_MOTOR_HOMESPEED * CAROUSEL_MOTOR_MICROSTEP);
  gantryMotorY.setSpeed(GANTRY_MOTORY_HOMESPEED * GANTRY_MOTORY_MICROSTEP);
  gantryMotorZ1.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);
  gantryMotorZ2.setSpeed(GANTRY_MOTORZ_HOMESPEED * GANTRY_MOTORZ_MICROSTEP);

  while (!carousel_homed || !gantryY_homed || !gantryZ_homed) {
    if (!carousel_homed) {
      if (digitalRead(Carousel_Motor_LimitSwitch_Pin) == LOW) {
        carousel_homed = true;
        carouselMotor.stop();
        carouselMotor.setCurrentPosition(0);
      }
      else { carouselMotor.runSpeed(); }
    }

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
void moving(long carousel_target, long gantryY_target, long gantryZ_target, int servo_state){

  bool need_to_reset_carousel = false;  // Biến kiểm tra có cần reset vị trí carousel hay không
  bool pushed_limitswitch = false;      // Biến kiểm tra đã chạm công tắc hành trình hay chưa
  bool is_moving_forward = (carousel_target >= 0);  // Xác định hướng di chuyển của carousel

  // Thiết lập vị trí mục tiêu cho các động cơ
  if (is_moving_forward) {
    carouselMotor.moveTo(INF);
    if (carouselMotor.currentPosition() > carousel_target)  { need_to_reset_carousel = true; }
  }
  else{
    carouselMotor.moveTo(-INF);
    if (carouselMotor.currentPosition() < abs(carousel_target)) {
      need_to_reset_carousel = true;
    }
  }
  gantryMotorY.moveTo(gantryY_target);
  gantryMotorZ1.moveTo(gantryZ_target);
  gantryMotorZ2.moveTo(gantryZ_target);

  // Bắt đầu di chuyển các động cơ
  while (true) {
    if (digitalRead(Carousel_Motor_LimitSwitch_Pin) != last_switch_state) {
      last_switch_state = digitalRead(Carousel_Motor_LimitSwitch_Pin);
      last_debounce_time = millis();
    }
    if ((millis() - last_debounce_time) > debounce_delay) {
      if (last_switch_state == HIGH && !is_moving_forward) {
        carouselMotor.setCurrentPosition(MAX_CAROUSEL_POS);
        pushed_limitswitch = true;
      }
      else if (last_switch_state == LOW && carousel_target > 0) {
        carouselMotor.setCurrentPosition(0);
        pushed_limitswitch = true;
      }
    }
    // Bien kiểm tra trạng thái hoàn thành của các động cơ
    bool gantry_is_done = (gantryMotorY.distanceToGo() == 0) && (gantryMotorZ1.distanceToGo() == 0);
    bool carousel_is_done = false;

    if (need_to_reset_carousel) {
      if (pushed_limitswitch) {
        if (is_moving_forward) { carousel_is_done = (carouselMotor.currentPosition() >= carousel_target); }
        else {                   carousel_is_done = (carouselMotor.currentPosition() <= abs(carousel_target)); }
      }
    }
    else{
      if (is_moving_forward) { carousel_is_done = (carouselMotor.currentPosition() >= carousel_target);}
      else {                   carousel_is_done = (carouselMotor.currentPosition() <= abs(carousel_target)); }
    }

    if (gantry_is_done && carousel_is_done) { break; }

    if (!gantry_is_done) {
      gantryMotorY.run();
      gantryMotorZ1.run();
      gantryMotorZ2.run();
    }

    if (!carousel_is_done) { carouselMotor.run(); }
  }
  // Dừng tất cả các động cơ
  carouselMotor.stop();
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

      long carousel_target = atol(strtok(command_buffer, ":"));
      long gantryY_target = atol(strtok(NULL, ":"));
      long gantryZ_target = atol(strtok(NULL, ":"));
      int servo_state = atoi(strtok(NULL, ":"));
      moving(carousel_target, gantryY_target, gantryZ_target, servo_state);
    }
    Serial.println("DONE:");
  }
}
