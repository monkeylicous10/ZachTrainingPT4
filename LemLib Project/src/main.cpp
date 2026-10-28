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

lemlib::TrackingWheel lHori(&hori, lemlib::Omniwheel::NEW_2, sqrt(3.94));
lemlib::TrackingWheel lVert(&vert, lemlib::Omniwheel::NEW_2, sqrt(12.89));

lemlib::Drivetrain driveTrain(&lmg,&rmg,10,lemlib::Omniwheel::NEW_325,450,2);

lemlib::ControllerSettings straight(10,0,10,5,0.5,4000,1,6000,30);
lemlib::ControllerSettings turn(2,0,2,3,.5,3000,1,5000,10);

lemlib::OdomSensors sensors(&lHori, nullptr,&lVert,nullptr,&imu);

lemlib::Chassis rob (driveTrain, straight, turn, sensors);

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
	rob.calibrate();
	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", rob.getPose().x); // x
            pros::lcd::print(1, "Y: %f", rob.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", rob.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

float curve(float input, float expo = 1.88) {
    float y = std::copysign(1,input);
    return std::pow(fabs(input/127.0f),expo) * y * 127.0f;
}

void opcontrol() {
	while(1) {
	rmg.move(curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)));
	lmg.move(curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
	pros::delay(20);
	}
}