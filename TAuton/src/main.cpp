#include "main.h"
#include <cmath>
#include "lemlib/api.hpp"
#include "pros/motors.hpp"
ASSET(firstpath_json);


pros::Controller master(pros::E_CONTROLLER_MASTER);

// Individual motors (with correct reversed flags)
pros::Motor right_F1(-20); 
pros::Motor right_F2(19);  
pros::Motor left_F1(11);   
pros::Motor left_F2(-12);    
pros::Motor right_R1(-17);
pros::Motor right_R2(-18);
pros::Motor left_R1(13);    //1 = 11W , 2 = 5W 
pros::Motor left_R2(14);


pros::Motor intake(1);
pros::Motor indexer(2);
pros::Motor topintake(3);

// Groups (each side)
pros::MotorGroup right_F({-20,19});
pros::MotorGroup left_F({11,-12});
pros::MotorGroup right_R({-17,-18});
pros::MotorGroup left_R({13,14});

pros::MotorGroup left_side({
    11,   // FL
    -12,  // FL2
    13,   // BL
    14    // BL2
});

// RIGHT SIDE
pros::MotorGroup right_side({
    -20,  // FR (reversed)
    19,   // FR2
    -17,  // BR (reversed)
    -18   // BR2 (reversed)
});
pros::Imu imu(10);

lemlib::Drivetrain drivetrain(&left_side, &right_side,
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

// linear settings 
lemlib::ControllerSettings linearSettings(
    10.0f,  // kP
    0.0f,   // kI (start at 0)
    3.0f,   // kD
    3.0f,   // windupRange (anti-windup)
    0.25f,   // smallError (inches)
    100.0f, // smallErrorTimeout (ms)
    1.0f,   // largeError (inches)
    500.0f, // largeErrorTimeout (ms)
    5.0f    // slew (max accel)
);

// Angular settings
lemlib::ControllerSettings angularSettings(
    2.0f,   // kP
    0.0f,   // kI
    10.0f,  // kD
    0.0f,   // windupRange
    1.0f,   // smallError (degrees)
    100.0f, // smallErrorTimeout (ms)
    3.0f,   // largeError (degrees)
    500.0f, // largeErrorTimeout (ms)
    5.0f    // slew
);


 lemlib::Chassis chassis(drivetrain,linearSettings,angularSettings, sensors);




// Group for ALL wheels
pros::MotorGroup allwheels({
    -11, -12, -1, -2, 20, 19, 9, 10 
});


//pros::adi::DigitalOut flap(4);

pros::adi::DigitalOut scraper(5);


double circumference = 2 * 3.14159; // wheel circumference in inches
// bool flapopen = false;
bool scraperon = false; 

// --- Function Declarations ---
void drivertrain();
void move(double degree, double length);
void turn(double degree);
void autonomous();
// --- Main ---
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    chassis.calibrate();
    
}
//autonomous code
void autonomous() {
    chassis.setPose(-63.097, -8.635, 12.545);
    chassis.follow(firstpath_json,true, false);
    chassis.waitUntilDone();
}    

void disabled() {}
void competition_initialize() {}




// --- Drivetrain for Mecanum (example) ---
void drivertrain(void) {
    while (true) {
        double vertical = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double spin = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        double strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        double maxval = std::max({fabs(vertical + strafe +spin), fabs(vertical + strafe -spin),fabs(vertical - strafe + spin), fabs(vertical - strafe -spin), 1.0});


        double topleft = vertical + strafe;
        double topright = vertical - strafe;

        right_F.move((topright - spin)/maxval);
        left_R.move((topright + spin)/maxval);
        right_R.move((topleft - spin)/maxval);
        left_F.move((topleft + spin)/maxval);

        // drive part of drivetrain

    //R1
    if (master.get_digital(DIGITAL_R1)) {
        intake.move(127);
        topintake.move(127);
        //if (!flapopen) {
        //flap.set_value(true);}
        //flapopen = true;
        //if (flapopen) {
        //flap.set_value(false);}
        //flapopen = false;
    } else {
        
        intake.move(0);
        topintake.move(0);
    }

    //R2
    if (master.get_digital(DIGITAL_R2)) {
        indexer.move(127);
        intake.move(127);
        topintake.move(127);
    } else {
        indexer.move(0);
        //move all the indexes, when in
        // handled above
    }

    //L1
    if (master.get_digital(DIGITAL_L1)) {
        intake.move(-127);
        indexer.move(127);
    } else {
        // stop if not pressed
        // 
        if (!master.get_digital(DIGITAL_R2) &&
            !master.get_digital(DIGITAL_L2)) {
            indexer.move(0);
        }
        if (!master.get_digital(DIGITAL_R1) &&
            !master.get_digital(DIGITAL_R2) &&
            !master.get_digital(DIGITAL_L2)) {
            intake.move(0);
        }
    }

    //L2
    if (master.get_digital(DIGITAL_L2)) {
        indexer.move(127);
        intake.move(127);
        topintake.move(-127);
    } else {
        if (!master.get_digital(DIGITAL_R2) &&
            !master.get_digital(DIGITAL_L1)) {
            indexer.move(0);
        }
        if (!master.get_digital(DIGITAL_R1) &&
            !master.get_digital(DIGITAL_R2) &&
            !master.get_digital(DIGITAL_L1)) {
            intake.move(0);
        }
        if (!master.get_digital(DIGITAL_R1) &&
            !master.get_digital(DIGITAL_R2)) {
            topintake.move(0);
        }
    }

    // --- Up (scraper) ---
    if (master.get_digital(DIGITAL_UP)) {
        if (!scraperon) {
        scraper.set_value(true);}
        scraperon = true;
        if (scraperon) {
        scraper.set_value(false);}
        scraperon = false;
    }

    pros::delay(20);
}



        
    
}

void opcontrol() {
    
        // Arcade drive
        drivertrain();
    
}
int main() {
    pros::Task autonomous_task([] {
        autonomous();
    });
    pros::Task opcontrol_task([] {
        opcontrol();
    });
    
    initialize();
    

    while (true) {
        pros::delay(20);
    }
}