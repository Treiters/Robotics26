#include "main.h"
#include <cmath>
#include <algorithm>
	
double circumference (2*3.14159); //wheel circumference in inches
void Davidequestionablecode() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor right_F1(11); // 5.5W motor
	right_F1.set_gearing(pros::E_MOTOR_GEARSET_18); // 200 RPM
	pros::Motor right_F2(-12); // 11W motor 
	pros::Motor left_F1(20); // 5.5W motor
	pros::Motor left_F2(-2); // 11W motor
	pros::Motor right_R1(20); // 5.5W motor
	pros::Motor right_R2(19); // 11W motor
	pros::Motor left_R1(-9); // 5.5W motor	
	pros::Motor left_R2(10); // 11W motor

// Create motor groups from the individual motors
	pros::MotorGroup right_F ({right_F1, right_F2});
	pros::MotorGroup left_F ({left_F1, left_F2});
	pros::MotorGroup right_R ({right_R1, right_R2});
	pros::MotorGroup left_R ({left_R1, left_R2});
	pros::MotorGroup allwheels ({right_F1, right_F2, left_F1, left_F2, right_R1, right_R2, left_R1, left_R2});	
	double circumference (2*3.14159); //wheel circumference in inches
	opcontrol();
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	drivetrain();
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		//left_mg.move(dir - turn);                      // Sets left motor voltage
		//right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}
/**
 * Things to declare and port 
 * controller - controller1
 * Forward left motor - LeftMotorsF 11, -12
 * Rear right motor - RightMotorsR, 9,10
 * Forward right motor - RightotorsF 1, -2
 * Rear left motor - LeftMotorsR 20, 19
 * 
 */
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


void move(double degree, double length) {
  
  double radians = degree*3.1415/180;

  double y = sin(radians);
  double x = cos(radians);


  double maxval = std::max(abs(x+y), abs(x-y),1);


  double frontleft = y + x;
  double frontright = y - x;
  double frontleft = frontleft/ maxval;
  double frontright = frontright/ maxval;


  double spins = length/ circumference;
  while (true) {

	left_F.move_velocity(50 * frontleft);   
	right_R.move_velocity(50 * frontleft);

	right_F.move_velocity(50 * frontright);
	left_R.move_velocity(50 * frontright);

	// Rotate group by "spins" revolutions at 100 RPM
	allwheels.move_relative(spins * 360, 100); 
  }}

void turn(double degree) {
	double horizontal = degree*circumference;
	right_F.move(-horizontal);
    left_R.move(horizontal);
    right_R.move(-horizontal);
    left_F.move(horizontal);
}