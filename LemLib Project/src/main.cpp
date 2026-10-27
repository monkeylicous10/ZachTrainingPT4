#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

pros::Controller roller (pros::E_CONTROLLER_MASTER);
pros::MotorGroup lift({-21,2});
pros::MotorGroup rmg ({18,19,20}, pros::MotorGearset::blue);
pros::MotorGroup lmg ({-11,-12,-13}, pros::MotorGearset::blue);
pros::adi::Pneumatics claw ('A', false);
pros::adi::Pneumatics tilt ('B',false);
pros::Rotation hori (1);
pros::Rotation vert (15);
pros::Imu imu (14);
pros::Distance paul (10);

lemlib::TrackingWheel lHori(&hori, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel lVert(&vert, lemlib::Omniwheel::NEW_2, -2.5);



lemlib::Drivetrain driveTrain(&lmg,&rmg,10,lemlib::Omniwheel::NEW_325,450,2);

lemlib::ControllerSettings straight(10,0,10,5,0.5,4000,1,6000,30);
lemlib::ControllerSettings turn(2,0,2,3,.5,3000,1,5000,10);

lemlib::OdomSensors sensors(&lHori, nullptr,&lVert,nullptr,&imu);

lemlib::Chassis chassis(driveTrain, straight, turn, sensors);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	chassis.calibrate();
	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}