#include "main.h"
#include <cmath>
#include <algorithm>

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Individual motors (with correct reversed flags)
pros::Motor right_F1(-11); 
pros::Motor right_F2(12);  
pros::Motor left_F1(-20);   
pros::Motor left_F2(2);    
pros::Motor right_R1(-20);
pros::Motor right_R2(-19);
pros::Motor left_R1(9);    //1 = forward front motor or below rear motor
pros::Motor left_R2(10);

// Groups (each side)
pros::MotorGroup right_F({-11,-12});
pros::MotorGroup left_F({-1,-2});
pros::MotorGroup right_R({20,19});
pros::MotorGroup left_R({-9,-10});

// Group for ALL wheels
pros::MotorGroup allwheels({
    -11, -12, -1, -2, 20, 19, 9, 10 
});

double circumference = 2 * 3.14159; // wheel circumference in inches

// --- Function Declarations ---
void drivetrain();
void move(double degree, double length);
void turn(double degree);

// --- Main ---
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    while (true) {
        // Arcade drive
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        left_F.move(dir - turn);
        left_R.move(dir - turn);
        right_F.move(dir + turn);
        right_R.move(dir + turn);

        pros::delay(20);
    }
}

// --- Drivetrain for Mecanum (example) ---
void drivetrain(void) {
    while (true) {
        int vertical = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int horizontal = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        double topleft = vertical + strafe;
        double topright = vertical - strafe;

        right_F.move(topright - horizontal);
        left_R.move(topright + horizontal);
        right_R.move(topleft - horizontal);
        left_F.move(topleft + horizontal);

        pros::delay(20);
    }
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
