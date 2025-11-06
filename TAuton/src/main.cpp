#include "main.h"
#include "lemlib/api.hpp"

void autonomous() {
    printf("Autonomous starting...\n");
    
    // Path 1: Drive forward to first position (straight line)
    chassis.follow({
        {-62.446, 17.202, 0},  // Start point
        {-45.79, 17.202, 0}    // End point
    }, 15, 3000);
    
    // Path 2: Curved path - START BOTH INTAKE AND INDEXER to collect rings
    intake.move(127);           // Start intake forward
    indexer.move(127);          // Start indexer forward
    
    chassis.follow({
        {-45.79, 17.202, 0},
        {-41.813, 38.581, 0},
        {-22.920, 9.993, 0}
    }, 15, 4000);
    
    // Stop both after collection
    intake.move(0);
    indexer.move(0);
    
    // Path 3: Move to scoring position
    chassis.follow({
        {-22.92, 9.993, 0},
        {-15.462, 15.462, 0}
    }, 15, 2000);
    
    // Path 4: Final positioning near goal
    chassis.follow({
        {-15.462, 15.462, 0},
        {-13.225, 13.225, 0}
    }, 15, 1000);
    
    // Score: Reverse BOTH intake and indexer to outtake into bottom goal
    intake.move(-127);          // Reverse intake to score
    indexer.move(-127);         // Reverse indexer to score
    pros::delay(2000);          // Run for 2 seconds to score
    intake.move(0);             // Stop intake
    indexer.move(0);            // Stop indexer
    
    printf("Autonomous complete!\n");
}

// Motor groups for drive
pros::Motor left_front_11w(11);
pros::Motor left_front_5w(-12);
pros::Motor right_front_11w(-20);
pros::Motor right_front_5w(19);
pros::Motor left_rear_11w(13);
pros::Motor left_rear_5w(14);
pros::Motor right_rear_11w(-18);
pros::Motor right_rear_5w(-17);

// Create motor groups for LemLib
pros::Motor_Group left_motors({left_front_11w, left_front_5w, left_rear_11w, left_rear_5w});
pros::Motor_Group right_motors({right_front_11w, right_front_5w, right_rear_11w, right_rear_5w});

// Mechanism motors
pros::Motor intake(1);
pros::Motor indexer(2);
pros::Motor top5(3);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Sensors for odometry (YOU NEED TO ADD THESE TO YOUR ROBOT)
// If you don't have these sensors yet, you can use basic open-loop control
pros::Imu imu(10);  // Change port number to match your IMU
pros::Rotation horizontal_encoder(7);  // Optional: for better tracking
pros::Rotation vertical_encoder(8);    // Optional: for better tracking

// Drivetrain configuration
lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                               12.75,  // track width (inches) - MEASURE YOUR ROBOT
                               lemlib::Omniwheel::NEW_275,  // wheel diameter
                               360,    // drive motor rpm (all motors on blue cartridge = 200rpm, adjust if different)
                               2);     // horizontal drift (tune this)

// Odometry configuration
lemlib::OdomSensors sensors(nullptr,  // vertical tracking wheel (use nullptr if not using)
                            nullptr,  // vertical tracking wheel 2 (use nullptr if not using)
                            nullptr,  // horizontal tracking wheel (use nullptr if not using)
                            nullptr,  // horizontal tracking wheel 2 (use nullptr if not using)
                            &imu);    // inertial sensor

// PID controllers (tune these values for your robot)
lemlib::ControllerSettings lateral_controller(10,   // kP
                                               0,    // kI
                                               3,    // kD
                                               3,    // anti-windup
                                               1,    // small error range (inches)
                                               100,  // small error timeout (ms)
                                               3,    // large error range (inches)
                                               500,  // large error timeout (ms)
                                               20);  // maximum acceleration (slew)

lemlib::ControllerSettings angular_controller(2,    // kP
                                               0,    // kI
                                               10,   // kD
                                               3,    // anti-windup
                                               1,    // small error range (degrees)
                                               100,  // small error timeout (ms)
                                               3,    // large error range (degrees)
                                               500,  // large error timeout (ms)
                                               0);   // maximum acceleration (slew)

// Create chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// Constants
constexpr int DEADZONE = 10;
constexpr int MAX_SPEED = 127;

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "PROS + LemLib Initialized");
    
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
    
    // Calibrate IMU
    pros::lcd::set_text(2, "Calibrating IMU...");
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    pros::lcd::set_text(2, "IMU Calibrated!");
    
    // Initialize chassis
    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    printf("Autonomous starting...\n");
    
    // Example: Move forward 24 inches at max speed
    chassis.moveToPoint(0, 24, 2000);
    
    // Example: Turn to 90 degrees
    chassis.turnToHeading(90, 1000);
    
    // Example: Follow a path from Jerry.io
    // After exporting from Jerry.io, you'll have coordinates like:
    /*
    chassis.follow({
        {0, 0, 0},      // x, y, heading
        {24, 0, 0},
        {24, 24, 90},
        {0, 24, 180}
    }, 15, 2000);  // timeout, async
    */
    
    // Add your Jerry.io paths here
    // You can also run intake/indexer during autonomous:
    // intake.move(127);
    // chassis.moveToPoint(12, 12, 2000);
    // intake.move(0);
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
        // Manual control using tank drive
        int left = master.get_analog(ANALOG_LEFT_Y);
        int right = master.get_analog(ANALOG_RIGHT_Y);
        
        // Drive with arcade control (for testing)
        chassis.arcade(master.get_analog(ANALOG_LEFT_Y), 
                      master.get_analog(ANALOG_RIGHT_X));
        
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
