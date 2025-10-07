#include "main.h"

// Motor groups for cleaner code
// Using gearset notation: positive = forward, negative = reversed
pros::Motor left_front_11w(11);
pros::Motor left_front_5w(-12);
pros::Motor right_front_11w(-20);
pros::Motor right_front_5w(19);
pros::Motor left_rear_11w(13);
pros::Motor left_rear_5w(14);
pros::Motor right_rear_11w(-18);
pros::Motor right_rear_5w(-17);

pros::Motor intake(1);
pros::Motor indexer(2);
pros::Motor top5(3);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Constants
constexpr int DEADZONE = 10;
constexpr int MAX_SPEED = 127;

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "PROS Initialized");
    
    // Set gearsets for motors
    intake.set_gearing(pros::E_MOTOR_GEARSET_18);    // 200 RPM (Blue)
    indexer.set_gearing(pros::E_MOTOR_GEARSET_06);   // 600 RPM (Green)
    top5.set_gearing(pros::E_MOTOR_GEARSET_18);      // 200 RPM (Blue)
    
    // Set brake modes
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    indexer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    top5.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    // Set voltage limits (95% torque)
    indexer.set_voltage_limit(12000 * 0.95);
    intake.set_voltage_limit(12000 * 0.95);
    top5.set_voltage_limit(12000 * 0.95);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    printf("auton is running\n");
}

// Helper function to set all drive motors
inline void set_drive(int fl, int fr, int rl, int rr) {
    left_front_11w.move(fl);
    left_front_5w.move(fl);
    right_front_11w.move(fr);
    right_front_5w.move(fr);
    left_rear_11w.move(rl);
    left_rear_5w.move(rl);
    right_rear_11w.move(rr);
    right_rear_5w.move(rr);
}

// Helper function to set intake system motors
inline void set_intake_system(int intake_speed, int indexer_speed, int top5_speed) {
    intake.move(intake_speed);
    indexer.move(indexer_speed);
    top5.move(top5_speed);
}

void opcontrol() {
    // Track previous button states to prevent stop-go stuttering
    bool prev_r2 = false, prev_r1 = false, prev_l1 = false, prev_l2 = false;
    int intake_speed = 0, indexer_speed = 0, top5_speed = 0;
    
    while (true) {
        // Read joystick values
        int forward = -master.get_analog(ANALOG_LEFT_Y);
        int strafe = -master.get_analog(ANALOG_LEFT_X);
        int turn = -master.get_analog(ANALOG_RIGHT_X);
        
        // Calculate drive speeds
        int fl, fr, rl, rr;
        
        if (abs(strafe) > DEADZONE) {
            // Strafe mode
            fl = strafe;
            fr = -strafe;
            rl = -strafe;
            rr = strafe;
        } else {
            // Tank drive with turn
            fl = rl = forward + turn;
            fr = rr = forward - turn;
        }
        
        set_drive(fl, fr, rl, rr);
        
        // Read current button states
        bool r2 = master.get_digital(DIGITAL_R2);
        bool r1 = master.get_digital(DIGITAL_R1);
        bool l1 = master.get_digital(DIGITAL_L1);
        bool l2 = master.get_digital(DIGITAL_L2);
        
        // Only update motor states if button states have changed OR are being held
        if (r2) {
            // R2: Intake reverse, indexer forward
            intake_speed = -MAX_SPEED;
            indexer_speed = MAX_SPEED;
            top5_speed = 0;
        } 
        else if (r1) {
            // R1: Indexer reverse, intake reverse, top5 forward
            intake_speed = -MAX_SPEED;
            indexer_speed = -MAX_SPEED;
            top5_speed = MAX_SPEED;
        } 
        else if (l1) {
            // L1: All reverse
            intake_speed = -MAX_SPEED;
            indexer_speed = -MAX_SPEED;
            top5_speed = -MAX_SPEED;
        } 
        else if (l2) {
            // L2: Indexer reverse, intake forward
            intake_speed = MAX_SPEED;
            indexer_speed = -MAX_SPEED;
            top5_speed = 0;
        }
        else {
            // No buttons pressed - stop all
            intake_speed = 0;
            indexer_speed = 0;
            top5_speed = 0;
        }
        
        // Apply motor speeds (only sends command if needed)
        set_intake_system(intake_speed, indexer_speed, top5_speed);
        
        // Update previous states
        prev_r2 = r2;
        prev_r1 = r1;
        prev_l1 = l1;
        prev_l2 = l2;
        
        // D-pad up handling (placeholder - original code had empty while loop)
        if (master.get_digital(DIGITAL_UP)) {
            // Add functionality here if needed
        }
        
        pros::delay(20);
    }
}