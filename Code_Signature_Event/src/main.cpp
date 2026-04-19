#include "main.h"
#include "auton.h"
#include "auton_selector.h"
#include "monte.h"
#include "lemlib/api.hpp" 
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"  
#include <future>
#include <cmath>
#include <cstdio>


// ========== GLOBAL STATE ==========
bool calibrate = false;

// === Motors ===
pros::MotorGroup leftMotors({12, -13, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-17, 18, 19}, pros::MotorGearset::blue);
pros::Motor motorTrai(7, pros::MotorGearset::blue);
pros::Motor motorPhai(-9, pros::MotorGearset::blue);

// === Pneumatics ===
pros::adi::Pneumatics stage('C', true);
pros::adi::Pneumatics descoreCenter('E', true); 
pros::adi::Pneumatics counterLoader('B', false);
pros::adi::Pneumatics descoreLeft('A', false);
pros::adi::Pneumatics odoLift('G', false); // SET BANG TRUE NEU DRIVER SKILL
pros::adi::Pneumatics blockblock('F', true);

// === Sensors ===
pros::Rotation horizontal_encoder(-8);
pros::Rotation vertical_encoder(16);
pros::Imu imu(2);
pros::Optical colorSensor(4);

// === Distance Sensors for Monte Carlo Localization ===
#include "monte_config.h"
pros::Distance dNorth(MCL_SENSOR_NORTH_PORT);  // North-facing sensor
pros::Distance dSouth(MCL_SENSOR_SOUTH_PORT);  // South-facing sensor
pros::Distance dEast(MCL_SENSOR_EAST_PORT);    // East-facing sensor
pros::Distance dWest(MCL_SENSOR_WEST_PORT);     // West-facing sensor 

// === Tracking Wheels ===
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.5);

// === Odom sensors ===
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// === Drivetrain ===
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.4, lemlib::Omniwheel::NEW_325, 450, 2);

// === PID Controllers ===
lemlib::ControllerSettings lateral_controller(14, 0, 91.5, 0, 0, 0, 0, 0, 0); // 8 41 | 14 87 | 14 91.5
lemlib::ControllerSettings angular_controller(4, 0, 31.35, 0, 0, 0, 0, 0, 0); // 4 30 | 4 28.5 | 4 32.35

// === Input Curves ===
// ExpoDriveCurve(deadband, minOutput, curve)
// - deadband: vùng input được coi là 0 (deadzone)
// - minOutput: output tối thiểu khi input vượt qua deadband
// - curve: độ cong (1.0 = linear, >1.0 = cong hơn, input nhỏ sẽ chậm hơn)
// Giá trị cao hơn (1.1-1.2) sẽ làm input nhỏ chậm hơn đáng kể
// lemlib::ExpoDriveCurve throttle_curve(3, 5, 1.12); // Tăng từ 1.019 lên 1.12 để chậm hơn ở input nhỏ
// lemlib::ExpoDriveCurve steer_curve(3, 5, 1.12); // Đồng bộ với throttle, tăng từ 1.05 lên 1.12

// === Chassis ===
// lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// === Controller ===
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ========== AUTON SELECTOR ==========
AutonSelector auton_selector;

// ========== MANUAL AUTON SELECTION ==========
// Set this to nullptr to use selector, or set to a function pointer to override selector
// Example: manualAutonFunction = &leftcenterdescore;
// Available autons:
// - &leftdescore7bloc
// - &leftfastdescore
// - &leftcenterdescore
// - &leftlongcenter2
// - &rightdescore7bloc
// - &rightdescore9bloc
// - &skillz
// - &testodo11
// - &awp
AutonSelector::routine_action_t manualAutonFunction = &skillz; // nullptr = use selector, or set to function pointer

void disabled() {
    // Update selector while disabled
    while (pros::competition::is_disabled()) {
        auton_selector.update();
        pros::delay(20);
    }
}

void competition_initialize() {
    // Initialize selector for competition
    auton_selector.initialize();
    while (pros::competition::is_disabled()) {
        auton_selector.update();
        pros::delay(20);
    }
}


// ========== HELPER STATES ==========
bool isSkillMode = false;  // rememberrr this
bool boostMode = false;

bool stageState = false;
bool gripperState = false;
bool counterLoaderState = false;
bool punchGoalState = false;
bool descoreLeftState = false;
bool descoreCenterState = false;
bool odoLiftState = false;
bool doubleParkState = false;
bool sequenceState = false;
bool colorFilterEnabled = false;
int colorToReject = 0;
bool loaderMode = false;
double speedAbove;
double speedUnder;
bool colorSortEnabled = false;
bool useManualColorSort = true;
AutonSelector::Color manualColorToSort = AutonSelector::Color::RED;
// Intake mode values:
// 0 = Stop motors (off)
// 1 = Blockblock mode (intake with blockblock engaged)
// 2 = Long goal mode (intake for long goal, blockblock off)
// 3 = Center goal mode (intake for center goal, blockblock on)
// 4 = Reverse/outtake mode (motorTrai reverse, motorPhai forward)
int intakeMode = 0;
int intakeSpeed = 127;  // 0-127, used by intake task when applying intakeMode (set from auton)
int current_auton = 0;
bool odoLifted = false;
int currentStageMode = -1;
bool motorTraiActive = false;


// ========== HELPER FUNCTIONS ==========
bool checkBlockDetected() {
    int proximity = colorSensor.get_proximity();
    return (proximity > 200);
}

void reverseMotorsForDuration(int duration_ms) {
    int speed = 127;
    motorTrai.move(-speed);
    motorPhai.move(-speed);

    int delay_count = duration_ms / 25; 
    for (int i = 0; i < delay_count; i++) {
        pros::delay(25);
    }
    
    // Dừng motor sau khi đảo ngược xong
    // motorTrai.move(0);
    // motorPhai.move(0);
}

// ========== TASKS ==========
void intakeTaskFn() { 
    static bool lastR1State = false;
    static bool lastR2State = false;
    static bool lastYState = false;
    static bool lastL1State = false;
    static bool l1Active = false; 
    static bool isReversing = false;
    static int reverseTimer = 0;
    while (true) {
        int speed = 127;
        
        bool r1Held = false;
        bool r2Held = false;
        bool yHeld = false;
        bool l1Pressed = false;
        
        if (!pros::competition::is_autonomous()) {
            r1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            yHeld = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
            bool l1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
            l1Pressed = l1Current && !lastL1State; // Edge detection: chỉ true khi vừa nhấn
            if (l1Pressed) {
                l1Active = !l1Active; // Toggle state
            }
            lastL1State = l1Current;
        }
        
        bool r1JustPressed = r1Held && !lastR1State;
        bool r2JustPressed = r2Held && !lastR2State;
        bool r1JustReleased = !r1Held && lastR1State;
        bool r2JustReleased = !r2Held && lastR2State;
        bool yJustReleased = !yHeld && lastYState;
        
        if (r1JustReleased || r2JustReleased || yJustReleased) {
            isReversing = false;
            reverseTimer = 0;
            l1Active = false;
        }
        
        if (isReversing && (r1Held || r2Held)) {
            reverseTimer -= 25; // Giảm timer (25ms mỗi loop)
            if (reverseTimer <= 0) {
                isReversing = false;
                motorTrai.move(0);
                motorPhai.move(0);
            } else {
                motorTrai.move(-speed);
                motorPhai.move(-speed);
                pros::delay(25);
                lastR1State = r1Held;
                lastR2State = r2Held;
                lastYState = yHeld;
                continue; 
            }
        } else if (isReversing && !r1Held && !r2Held) {
            // Đang reverse nhưng R1/R2 đã bị thả, dừng ngay
            isReversing = false;
            reverseTimer = 0;
            motorTrai.move(0);
            motorPhai.move(0);
        }
        
        if ((r1JustPressed || r2JustPressed) && !pros::competition::is_autonomous()) {
            if (checkBlockDetected()) {
                isReversing = true;
                reverseTimer = 150;
                motorTrai.move(-speed);
                motorPhai.move(-speed);
                pros::delay(25);
                lastR1State = r1Held;
                lastR2State = r2Held;
                lastYState = yHeld;
                continue; 
            }
        }
        
        lastR1State = r1Held;
        lastR2State = r2Held;
        lastYState = yHeld;
        
        // Skill: R2 chậm (91) ngay từ đầu. R1 luôn full (127).
        int r2Speed = (isSkillMode && r2Held) ? 91 : speed;
        
        // Manual controls override intakeMode
        if (r1Held) {
            blockblock.set_value(false);
            motorTrai.move(speed);
            motorPhai.move(speed);
        } else if (r2Held) {
            blockblock.set_value(true);
            motorTrai.move(r2Speed);
            motorPhai.move(r2Speed);
        } else if (yHeld) {
            motorTrai.move(-speed);
            motorPhai.move(-speed);
        } else if (l1Active && !pros::competition::is_autonomous()) {
            // L1 override (toggle on press): Blockblock mode (intake with blockblock engaged)
            blockblock.set_value(true);
            stage.set_value(true);
            motorTrai.move(speed);
            motorPhai.move(speed);
        } else {
            // No manual controls - use intakeMode (y chang như cũ, vẫn dùng intakeSpeed)
            if (intakeMode == 0) {
                blockblock.set_value(true);
                stage.set_value(true);
                motorTrai.move(0);
                motorPhai.move(0);
            } else if (intakeMode == 1) {
                blockblock.set_value(true);
                stage.set_value(true);
                motorTrai.move(intakeSpeed);
                motorPhai.move(intakeSpeed);
            } else if (intakeMode == 2) {
                blockblock.set_value(false);
                stage.set_value(true);
                motorTrai.move(intakeSpeed);
                motorPhai.move(intakeSpeed);
            } else if (intakeMode == 3) {
                blockblock.set_value(true);
                stage.set_value(false);
                motorTrai.move(intakeSpeed);
                motorPhai.move(intakeSpeed);
            } else if (intakeMode == 4) {
                stage.set_value(true);
                blockblock.set_value(true);
                motorTrai.move(-intakeSpeed);
                motorPhai.move(-intakeSpeed);
            } else if (intakeMode == 5) {
                stage.set_value(false);
                blockblock.set_value(true);
                motorTrai.move(0);
                motorPhai.move(0);
            } else if (intakeMode == 6) {
                stage.set_value(false);
                blockblock.set_value(true);
                motorTrai.move(-intakeSpeed);
                motorPhai.move(-intakeSpeed);
            }
        }
        
        pros::delay(25);
    }
}

pros::Task intakeTask(intakeTaskFn);

void handleDrive() {
    // Đọc input từ controller
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    const int forwardDeadzone = 5;
    const int turnDeadzone = 5; 
    if (abs(leftY) < forwardDeadzone) leftY = 0;
    if (abs(rightX) < turnDeadzone) rightX = 0;
    
    if (isSkillMode && boostMode) {
        leftY = (int)(leftY * 81.0 / 127.0);
        rightX = (int)(rightX * 81.0 / 127.0);
    }
    
    chassis.arcade(leftY, rightX, false, 0.35);
}

void handleManualPneumatics() {
    // L1: toggle motorTrai (chỉ toggle khi nhấn 1 lần) + bật blockblock
    static bool lastL1State = false;
    bool l1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    if (l1Current && !lastL1State) {
        motorTraiActive = !motorTraiActive;
        blockblock.set_value(true);
    }
    lastL1State = l1Current;
    
    // Khi thả R1/R2/Y: tắt motorTrai (reset motorTraiActive)
    static bool lastR1State = false, lastR2State = false, lastYState = false;
    bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool yCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    
    if ((lastR1State && !r1Current) || (lastR2State && !r2Current) || (lastYState && !yCurrent)) {
        motorTraiActive = false;
    }
    
    // R1: giữ thì tắt blockblock, thả thì bật blockblock
    if (r1Current) {
        blockblock.set_value(false);
    } else if (!r1Current && lastR1State) {
        blockblock.set_value(true);
    }
    
    // R2: bật pneumatic stage khi giữ, tắt khi thả
    if (r2Current && !lastR2State) {
        stage.set_value(false);
        blockblock.set_value(true);
    } else if (!r2Current && lastR2State) {
        stage.set_value(true);
        blockblock.set_value(true);
    }
    
    lastR1State = r1Current;
    lastR2State = r2Current;
    lastYState = yCurrent;
    
    // L2: In skill mode toggle boost; otherwise toggle left descore
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (isSkillMode) {
            boostMode = !boostMode;
        } else {
            descoreLeftState = !descoreLeftState;
            descoreLeft.set_value(descoreLeftState);
        }
    }

    // B: toggle color sort
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        descoreCenterState = !descoreCenterState;
        descoreCenter.set_value(descoreCenterState);
    }


    // X: toggle odo lift (đổi từ UP sang X)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        odoLiftState = !odoLiftState;  
        odoLift.set_value(odoLiftState); 
    }

    // A: toggle double park
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        counterLoaderState = !counterLoaderState;
        counterLoader.set_value(counterLoaderState);
    }



    // LEFT: toggle boost (skill mode only)
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //     if (isSkillMode) {
    //         boostMode = !boostMode;
    //         controller.rumble(".");
    //     }
    // }

}

// Telemetry update task for autonomous
void telemetryTaskFn(void* param) {
    while (true) {
        auton_selector.update_telemetry();
        lv_timer_handler(); // Update LVGL display
        pros::delay(100); // Update every 100ms
    }
}

void autonomous() {
    auton_selector.show_telemetry();
    pros::Task telemetryTask(telemetryTaskFn, nullptr, "TelemetryTask");

    isSkillMode = (auton_selector.get_selected_name() == "Skillz");

    if (manualAutonFunction != nullptr) {
        manualAutonFunction();
    } else {
        auton_selector.run_auton();
    }
}

void initialize() {
    chassis.calibrate();
    // colorSensor.set_led_pwm(100);

    // Set brake mode for drivetrain
    // - pros::E_MOTOR_BRAKE_COAST: Motors coast when stopped (no resistance)
    // - pros::E_MOTOR_BRAKE_BRAKE: Motors brake when stopped (some resistance)
    // - pros::E_MOTOR_BRAKE_HOLD: Motors hold positiBRAKEon when stopped (strongest, best for precision)
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE); 

    auton_selector.initialize();
    

    // chassis.setPose(45, 0, 270);
    // startMCL(chassis); // Start MCL to correct odometry drift
}


// Skill mode opcontrol: chạy từng chu trình, DOWN = ngắt
static bool skillRunning = false;
static pros::Task* skillTask = nullptr;
static void skillTaskFnAll(void*) {
    odoLift.set_value(false);
    pros::delay(50);
    chutrinhtrai();
    quapark2();
    chutrinhphai();
    skillRunning = false;
}
static void skillTaskFnQuapark2(void*) {
    odoLift.set_value(false);
    pros::delay(50);
    resetposeskillxNorth(1);
    resetposeskilly(1);
    quapark2();
    skillRunning = false;
}
static void skillTaskFnChutrinhphai(void*) {
    // chassis.setPose(-29.5, -47.25, chassis.getPose().theta);
    // resetposeskilly(-1);
    resetposeskillxNorth(-1);
    resetposeskilly(-1);
    pros::delay(50);
    odoLift.set_value(false);
    chutrinhphai();
    skillRunning = false;
}
static void skillAbort() {
    intakeMode = 0;
    odoLift.set_value(true);
    chassis.cancelMotion();
    if (skillTask != nullptr) {
        skillTask->remove();
        skillTask = nullptr;
    }
    skillRunning = false;
}
static void skillStartAll() {
    skillRunning = true;
    skillTask = new pros::Task(skillTaskFnAll, nullptr, "skill_all");
}
static void skillStartQuapark2() {
    skillRunning = true;
    skillTask = new pros::Task(skillTaskFnQuapark2, nullptr, "skill_qp2");
}
static void skillStartChutrinhphai() {
    skillRunning = true;
    skillTask = new pros::Task(skillTaskFnChutrinhphai, nullptr, "skill_ctp");
}

// ========== OP CONTROL ==========
void opcontrol() {   
    
    auton_selector.show_telemetry();
    intakeMode = 0;
    
    int telemetryCounter = 0;
    int controllerDisplayCounter = 0;

    while (true) {
        if (isSkillMode) {
            // UP = chạy skill (cả 3 chu trình)
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && !skillRunning) {
                skillStartAll();
            }
            // DOWN = chỉ ngắt chu trình đang chạy
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && skillRunning) {
                skillAbort();
            }
            // LEFT = chạy quapark2
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && !skillRunning) {
                skillStartQuapark2();
            }
            // RIGHT = chạy chutrinhphai
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) && !skillRunning) {
                skillStartChutrinhphai();
            }
        }
        
        handleManualPneumatics();
        if (!skillRunning) handleDrive();
        
        // Update telemetry periodically (every 4 loops = ~100ms at 25ms delay)
        if (telemetryCounter % 4 == 0) {
            auton_selector.update_telemetry();
            lv_timer_handler(); // Update LVGL display
        }
        telemetryCounter++;
        
        // Update controller display periodically (every 2 loops = ~50ms)
        // Controller text update is slow, so don't update too frequently
        if (controllerDisplayCounter % 2 == 0) {
            lemlib::Pose pose = chassis.getPose();
            controller.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
            // controller.print(1, 0, "T:%.1f", pose.theta);
        }
        controllerDisplayCounter++;
        
        pros::delay(25);
    }
}

// void opcontrol() {   
    
//     manualAutonFunction();
    
    
// }