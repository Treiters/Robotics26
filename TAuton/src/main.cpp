#include "main.h"
#include <cmath>
#include <algorithm>
#include "lemlib/api.hpp"

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
pros::MotorGroup right_F({-11,-12});
pros::MotorGroup left_F({-1,-2});
pros::MotorGroup right_R({20,19});
pros::MotorGroup left_R({-9,-10});



pros::Imu imu(10);

lemlib::Drivetrain drivetrain(
    left_F, left_R, right_F, right_R, // For tank drives
    12.5, // wheel track in inches
    2,// wheel diameter
    800,
    lemlib::DriveType::X_DRIVE,

);
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

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, &imu);




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
void drivetrain();
void move(double degree, double length);
void turn(double degree);
void autonomous();
// --- Main ---
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
}
//autonomous code
 lemlib::Chassis chassis(drivetrain,linearController,angularController,sensors);
void autonomous() {
    std::vector<lemlib::Pose> path = {
        { -63.097, -8.635, 12.545 },
        { -61.359, -9.623, 21.172 },
        { -59.656, -10.671, 21.963 },
        { -57.955, -11.723, 21.748 },
        { -56.25, -12.768, 21.63 },
        { -54.537, -13.8, 21.527 },
        { -52.813, -14.814, 21.423 },
        { -51.076, -15.806, 21.305 },
        { -49.324, -16.771, 21.159 },
        { -47.555, -17.704, 20.971 },
        { -45.767, -18.599, 20.716 },
        { -43.954, -19.445, 20.554 },
        { -42.115, -20.229, 20.129 },
        { -40.243, -20.933, 19.081 },
        { -38.333, -21.524, 17.894 },
        { -36.38, -21.953, 16.008 },
        { -34.391, -22.126, 10.884 },
        { -32.399, -21.995, 22 },
        { -30.399, -21.977, 22 },
        { -28.399, -21.959, 22 },
        { -26.399, -21.942, 22 },
        { -24.399, -21.924, 22 },
        { -22.399, -21.907, 22 },
        { -20.399, -21.889, 22 },
        { -18.399, -21.871, 22 },
        { -16.399, -21.864, 17.226 },
        { -14.407, -22.003, 2.114 },
        { -13.374, -23.149, 0 },
        { -14.918, -24.401, 17.188 },
        { -16.657, -25.388, 20.1 },
        { -18.444, -26.285, 21.096 },
        { -20.253, -27.138, 21.351 },
        { -22.076, -27.96, 21.653 },
        { -23.908, -28.763, 21.747 },
        { -25.746, -29.553, 21.818 },
        { -27.587, -30.333, 21.875 },
        { -29.431, -31.108, 21.92 },
        { -31.277, -31.878, 21.958 },
        { -33.123, -32.646, 21.99 },
        { -34.971, -33.412, 21.981 },
        { -36.818, -34.179, 21.955 }
    };
    chassis.follow(path, true);
    chassis.waitUntilDone();
}
void disabled() {}
void competition_initialize() {}


void opcontrol() {
    while (true) {
        // Arcade drive
        drivetrain();
    }
}

// --- Drivetrain for Mecanum (example) ---
void drivetrain(void) {
    while (true) {
        int vertical = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int horizontal = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        double maxval = std::max({fabs(vertical + strafe +horizontal), fabs(vertical + strafe -horizontal),fabs(vertical - strafe + horizontal), fabs(vertical - strafe -horizontal), 1.0});


        double topleft = vertical + strafe;
        double topright = vertical - strafe;

        right_F.move((topright - horizontal)/maxval);
        left_R.move((topright + horizontal)/maxval);
        right_R.move((topleft - horizontal)/maxval);
        left_F.move((topleft + horizontal)/maxval);

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



        pros::delay(20);
    
}

// --- Move in direction by length ---
void move(double degree, double length) {
    double radians = degree * 3.1415 / 180;
    double y = sin(radians);
    double x = cos(radians);

    double maxval = std::max({fabs(x + y), fabs(x - y), 1.0});

    double frontleft = (y + x) / maxval;
    double frontright = (y - x) / maxval;

    double spins = length / circumference;

    // Move all wheels by target spins
    allwheels.move_relative(spins * 360, 100);
}

// --- Turn by degrees ---
void turn(double degree) {
    double horizontal = degree * circumference;
    right_F.move(-horizontal);
    right_R.move(-horizontal);
    left_F.move(horizontal);
    left_R.move(horizontal);
}
int main() {
    drivetrain();
    return 0;
}
