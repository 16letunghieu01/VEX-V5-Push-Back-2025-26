/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\ADMIN                                            */
/*    Created:      Thu Jan 23 2025                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Left1                motor         8               
// Left2                motor         7               
// Left3                motor         10              
// Right1               motor         4               
// Right2               motor         5               
// Right3               motor         2               
// Inertial             inertial      6               
// Controller1          controller                    
// InsideRoller         motor         3               
// Intake1              motor         9               
// Intake2              motor         1               
// PistonA              digital_out   A               
// PistonB              digital_out   B               
// PistonH              digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>  
#include <vector>
#include <cstdlib>
#include <string>

const int deadzone = 15;
using namespace vex;

//////////////////////////////////////////////////////////////////
////////////////////deadzone void////////////////////////////////
////////////////////////////////////////////////////////////////

int applyDeadzone(int value) {
  if (abs(value) < deadzone) {
    return 0; 
  }
  return value; 
}

//////////////////////////////////////////////////////////////////
////////////////////ladybrow value///////////////////////////////
////////////////////////////////////////////////////////////////

const int numStates = 3;
int states[numStates] = {0, 6000, 40000};
int currState = 0;
int target = 0;

double LBkp = 3;

void graphPID(std::vector<int> errorHistory, std::vector<float> powerHistory, int goal, float error, int time) {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawLine(0, 60, 480, 60);
  Brain.Screen.setPenWidth(1);
  Brain.Screen.setPenColor(green);

  Brain.Screen.setCursor(1, 1);
  Brain.Screen.clearLine(1);
  Brain.Screen.print(" Final Error: ");
  Brain.Screen.print(error);
  Brain.Screen.print("    Time: ");
  Brain.Screen.print(time);
  
  int minY = 60; 
  int maxY = 230; 
  int minX = 10; 
  int maxX = 470; 
  
  for (int i = 0; i < errorHistory.size() - 1; i++) { 
    int x = minX + (maxX - minX) * i / errorHistory.size(); 
    
    Brain.Screen.setPenColor(green);
    Brain.Screen.drawLine(x, minY + (float)errorHistory.at(i) / goal * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), minY + (float)errorHistory.at(i + 1) / goal * (maxY - minY));
    
    if (powerHistory.at(i) > 0) {
      Brain.Screen.setPenColor(orange);
    } else {
      Brain.Screen.setPenColor(yellow);
    }
    
    Brain.Screen.drawLine(x, maxY - std::abs(powerHistory.at(i)) * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), maxY - std::abs(powerHistory.at(i + 1)) * (maxY - minY));
  }
}

//////////////////////////////////////////////////////////////////
////////////////////turn PID void////////////////////////////////
////////////////////////////////////////////////////////////////

int turnPID(int turnDistance) {
  // float kP = 0.02;
  // float kI = 0.0011;
  // float kD = 0.015;

  // float kP = 0.021;
  // float kI = 0.0014;
  // float kD = 0.12;

  float kP = 0.027;
  float kI = 0.000855;
  float kD = 0.125;

  // float kP = 0.02835;
  // float kI = 0.00375;
  // float kD = 0.11;

  float error = 0;      
  float integral = 0;   
  float derivative = 0; 
  float prevError = 0;  
  float motorPower = 0;     
  float prevMotorPower = 0;

  float startDistance = Inertial.rotation(degrees);  

  std::vector<int> errorHistory; 
  std::vector<float> powerHistory;
  int currentTime = 0;

  while(true) {
    float currentDistance = startDistance - Inertial.rotation(degrees);

    error = turnDistance - currentDistance;

    if (error < 10 && error > -10) {
      integral += error;
    }

    derivative = error - prevError;

    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    if (motorPower > 1) motorPower = 1;
    if (motorPower < -1) motorPower = -1;

    float slewRate = 0.1f;
    if (motorPower > prevMotorPower + slewRate) motorPower = prevMotorPower + slewRate;
    if (motorPower < prevMotorPower - slewRate) motorPower = prevMotorPower - slewRate;

    Left1.spin(forward, -11 * motorPower, volt);
    // Left2.spin(forward, -11 * motorPower, volt);
    // Left3.spin(forward, -11 * motorPower, volt);
    // Right1.spin(forward, 11 * motorPower, volt);
    // Right2.spin(forward, 11 * motorPower, volt);
    // Right3.spin(forward, 11 * motorPower, volt);

    if (error > -1.5 && error < 1.5 && error - prevError > -0.1 && error - prevError < 0.1) {
      break;
    }

    prevMotorPower = motorPower;
    prevError = error;

    errorHistory.push_back(error);
    powerHistory.push_back(std::abs(motorPower));
    currentTime += 20;

    graphPID(errorHistory, powerHistory, turnDistance, error, currentTime);

    wait(20, msec);
  }
  Left1.stop();
  Left2.stop();
  Left3.stop();
  Right1.stop();
  Right2.stop();
  Right3.stop();

    return 0;
}


void intakeControl() {
  // --- Intake (L1: hút vào) ---
  if (Controller1.ButtonL1.pressing()) {
    Intake1.spin(forward, 100, pct);    // quay xuôi
    Intake2.spin(reverse, 100, pct);    // quay ngược
    InsideRoller.spin(forward, 100, pct);
  }
  // --- Intake đảo chiều (L2: nhả ra ngược) ---
  else if (Controller1.ButtonL2.pressing()) {
    Intake1.spin(reverse, 100, pct);
    Intake2.spin(forward, 100, pct);
    InsideRoller.spin(reverse, 100, pct);
  }
  // --- Outtake1 (R1: đẩy ra) ---
  else if (Controller1.ButtonR1.pressing()) {
    Intake1.spin(forward, 100, pct);
    Intake2.spin(reverse, 100, pct);
    InsideRoller.spin(reverse, 100, pct);
  }
  // --- Outtake2 (R2: InsideRoller đảo chiều, 2 intake cùng chiều) ---
  else if (Controller1.ButtonR2.pressing()) {
    Intake1.spin(forward, 100, pct);
    Intake2.spin(forward, 100, pct);
    InsideRoller.spin(reverse, 100, pct);
  }
  else {
    // Không bấm nút nào => dừng
    Intake1.stop(coast);
    Intake2.stop(coast);
    InsideRoller.stop(coast);
  }
}

//////////////////////////////////////////////////////////////////
////////////////////drive PID void////////////////////////////////
////////////////////////////////////////////////////////////////

int drivePID(int driveDistance) {
  float kP = 0.0015; // p la toc do???
  float kI = 0.00; // i
  float kD = 0.00381;

  float error = 0;      
  float integral = 0;  
  float derivative = 0; 
  float prevError = 0;  

  float motorPower = 0;     
  float prevMotorPower = 0; 

    std::vector<int> errorHistory; 
    std::vector<float> powerHistory;
    int currentTime = 0;

    Right1.setPosition(0, degrees);
    Right2.setPosition(0, degrees);
    Right3.setPosition(0, degrees);
    Left1.setPosition(0, degrees);
    Left2.setPosition(0, degrees);
    Left3.setPosition(0, degrees);

    while(true) {
    float currentDistance = (Right1.position(degrees) + Left1.position(degrees) + Right2.position(degrees) + Left2.position(degrees) + Right3.position(degrees) + Left3.position(degrees)) / 6;
    error = driveDistance - currentDistance;
    if (error < 200 && error > -200) {
      integral += error;
    }
    derivative = error - prevError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative); 
    if (motorPower > 1) motorPower = 1;
    if (motorPower < -1) motorPower = -1;
    float slewRate = 0.1f;
    if (motorPower > prevMotorPower + slewRate) motorPower = prevMotorPower + slewRate;
    if (motorPower < prevMotorPower - slewRate) motorPower = prevMotorPower - slewRate;
    Left1.spin(forward, 11 * motorPower, volt);
    Left2.spin(forward, 11 * motorPower, volt);
    Left3.spin(forward, 11 * motorPower, volt);
    Right1.spin(forward, 11 * motorPower, volt);
    Right2.spin(forward, 11 * motorPower, volt);
    Right3.spin(forward, 11 * motorPower, volt);
    if (error > -30 && error < 30 && error - prevError > -5 && error - prevError < 5) {
        break;
    }
    prevMotorPower = motorPower;
    prevError = error;

    errorHistory.push_back(error);
    powerHistory.push_back(std::abs(motorPower));
    currentTime += 20;

    graphPID(errorHistory, powerHistory, driveDistance, error, currentTime);
    
    wait(20, msec);
}

  Left1.stop();
  Left2.stop();
  Left3.stop();
  Right1.stop();
  Right2.stop();
  Right3.stop();

    return 0;
}

int driveNoPID(int driveDistance) {
    // Xác định hướng chạy: nếu driveDistance dương thì chạy tiến, âm thì lùi.
    int direction = (driveDistance >= 0) ? 1 : -1;
    // Công suất motor cố định (có thể điều chỉnh giá trị này cho phù hợp)
    float motorPower = 0.5 * direction;
    
    // Các biến kiểm tra sự tiến triển
    int noMovementCount = 0;
    const int movementThreshold = 5; // ngưỡng thay đổi encoder (đơn vị: độ)
    int lastDistance = 0;
    
    // Reset vị trí encoder của tất cả các motor
    Right1.setPosition(0, degrees);
    Right2.setPosition(0, degrees);
    Right3.setPosition(0, degrees);
    Left1.setPosition(0, degrees);
    Left2.setPosition(0, degrees);
    Left3.setPosition(0, degrees);
    
    while (true) {
        // Tính khoảng cách trung bình hiện tại (đơn vị: độ)
        int currentDistance = (Right1.position(degrees) + Right2.position(degrees) + Right3.position(degrees)
                              + Left1.position(degrees) + Left2.position(degrees) + Left3.position(degrees)) / 6;
        
        // Nếu đã đạt hoặc vượt qua khoảng cách yêu cầu thì thoát vòng lặp
        if ((direction == 1 && currentDistance >= driveDistance) ||
            (direction == -1 && currentDistance <= driveDistance)) {
            break;
        }
        
        // Kiểm tra xem robot có đang tiến bộ hay không
        if (abs(currentDistance - lastDistance) < movementThreshold) {
            noMovementCount++;
        } else {
            noMovementCount = 0;
        }
        
        // Nếu không có sự tiến triển (ví dụ hơn 1 giây, tương đương với 50 vòng lặp với delay 20ms)
        if (noMovementCount > 50) {
            break;
        }
        
        // Chạy các motor với công suất cố định
        Left1.spin(forward, 11 * motorPower, volt);
        Left2.spin(forward, 11 * motorPower, volt);
        Left3.spin(forward, 11 * motorPower, volt);
        Right1.spin(forward, 11 * motorPower, volt);
        Right2.spin(forward, 11 * motorPower, volt);
        Right3.spin(forward, 11 * motorPower, volt);
        
        lastDistance = currentDistance;
        wait(20, msec);
    }
    
    // Dừng các motor
    Left1.stop();
    Left2.stop();
    Left3.stop();
    Right1.stop();
    Right2.stop();
    Right3.stop();
    
    return 0;
}

// Biến trạng thái khí nén
bool pistonAState = false;
bool pistonBState = false;
bool pistonHState = false;

void pneumaticControl() {
  // --- Điều khiển khí nén cổng A bằng nút X ---
  if (Controller1.ButtonX.pressing()) {
    pistonAState = !pistonAState;         // đảo trạng thái
    PistonA.set(pistonAState);            // xuất tín hiệu
    while (Controller1.ButtonX.pressing()) wait(10, msec); // chống double toggle
  }

  // --- Điều khiển khí nén cổng B bằng nút A ---
  if (Controller1.ButtonA.pressing()) {
    pistonBState = !pistonBState;
    PistonB.set(pistonBState);
    while (Controller1.ButtonA.pressing()) wait(10, msec);
  }

  // --- Điều khiển khí nén cổng H bằng nút B ---
  if (Controller1.ButtonB.pressing()) {
    pistonHState = !pistonHState;
    PistonH.set(pistonHState);
    while (Controller1.ButtonB.pressing()) wait(10, msec);
  }
}


//////////////////////////////////////////////////////////////////
////////////////////control void/////////////////////////////////
////////////////////////////////////////////////////////////////

void driveCode() {
  int straight = Controller1.Axis3.value();
  int turn = Controller1.Axis1.value();

  straight = applyDeadzone(straight);
  turn = applyDeadzone(turn);

  if (straight > 70) {
    turn *= (((straight - 50) / 100) + 1);
  } else if (straight < -50) {
    turn *= (((-70 - straight) / 100) + 1);
  } 

  int left = straight + turn * 0.7; 
  int right = straight - turn * 0.7;

  Left1.spin(forward, left, percent);
  Left2.spin(forward, left, percent);
  Left3.spin(forward, left, percent);
  Right1.spin(forward, right, percent);
  Right2.spin(forward, right, percent);
  Right3.spin(forward, right, percent);
  }

//////////////////////////////////////////////////////////////////
////////////////////user control and auto void///////////////////
////////////////////////////////////////////////////////////////

void userControl() {
    while (true) {
      driveCode();
      intakeControl();   // điều khiển intake / outtake
      pneumaticControl();
  }
  wait(15,msec);
}

/////////////////////////////////////////////////////////////////////////////
///////////////////////////////help code auto///////////////////////////////
///////////////////////////////////////////////////////////////////////////

//      drivePID(x); tiến lên
//      drivePID(-x); lùi xuống
//      turnPID(x); quay trái
//      turnPID(-x); quay phải
//      bottomIntake.abcdef điều kiển intake
//      middleIntake.abcdef điều kiển intake
//      MogoMech.set( true or false ); điều khiển mogo mech
//      AutoStatex() x là gt từ 0-2 chỉnh vị trí lady brown
//      Arm.set( true or false ); điều kiển Arm
  
//////////////////////////////////////////////////////////////////
//////////////////// đừng xoá lộn nhe pls////////////////////////
////////////////////////////////////////////////////////////////

void testPID() {
  // 1️⃣ Bật intake liên tục từ đầu
  Intake1.spin(forward, 100, pct);
  Intake2.spin(reverse, 100, pct);
  InsideRoller.spin(forward, 100, pct);

  // 2️⃣ Chạy tới 600
  drivePID(1000);

  // 3️⃣ Chạy thêm 600 nữa
  drivePID(800);

  // 4️⃣ Tắt intake khi hoàn thành
  Intake1.stop();
  Intake2.stop();
  InsideRoller.stop();

  // 5️⃣ Dừng tất cả motor di chuyển để an toàn
  Left1.stop();
  Left2.stop();
  Left3.stop();
  Right1.stop();
  Right2.stop();
  Right3.stop();
}

competition Competition;

int main() {
  Inertial.calibrate();
  wait(100,msec);
  Competition.autonomous(testPID);
  // Competition.autonomous(autonomousPIDSkillrall);
  // Competition.autonomous(autoTurn);
  // Competition.autonomous(tuningPID);
  Competition.drivercontrol(userControl);
}

