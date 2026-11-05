#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include <sys/types.h>
pros::Controller roller (pros::E_CONTROLLER_MASTER);

pros::MotorGroup lift({-21,2},pros::MotorGearset::green);

pros::MotorGroup rmg ({18,19,20}, pros::MotorGearset::blue);

pros::MotorGroup lmg ({-11,-12,-13}, pros::MotorGearset::blue);

pros::adi::Pneumatics claw ('A', false);

pros::adi::Pneumatics tilt ('B',false);

pros::Rotation hori (1);

pros::Rotation vert (15);

pros::Imu ethan (14);

pros::Distance paul (10);

lemlib::TrackingWheel lHori(&hori, lemlib::Omniwheel::NEW_2, sqrt(3.94));

lemlib::TrackingWheel lVert(&vert, lemlib::Omniwheel::NEW_2, sqrt(12.89));

lemlib::Drivetrain driveTrain(&lmg,&rmg,13,lemlib::Omniwheel::NEW_325,450,2);

lemlib::ControllerSettings straight(12,0,18,4,.5,100,2,500,0);

lemlib::ControllerSettings turnP(5,0,45,3,1,100,3,500,0);

lemlib::OdomSensors sensors(&lVert, nullptr,&lHori,nullptr,&ethan);

lemlib::Chassis rob (driveTrain, straight, turnP, sensors);

//PID DRIVE CONST
constexpr float dkP = 13;
constexpr float dkI = .0f;
constexpr float dkD = 1.2;

//PID TURNING CONST
constexpr float turnkP = .865;
constexpr float turnkI = .0;
constexpr float turnkD = .4;

float wrapAngle(float angle) {
    while (angle < -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
}
float getLeftInches() {
    return lmg.get_position()/450 * (M_PI * 3.25);
}

float getRightInches() {
    return rmg.get_position()/450 * (M_PI * 3.25);
}

float getAvInches() {
    return (getLeftInches() + getRightInches())/2;
}
void tare() {
    rmg.tare_position();
    lmg.tare_position();
}
float liftgoal{};
float errora{},prevErrora{},integrala{},derivativea{},currenta = lift.get_position();
float liftPIDDR (float goal = liftgoal) {
    errora = (goal) - currenta;
    currenta = lift.get_position();
    integrala += errora;
    derivativea = errora - prevErrora;
    prevErrora = errora;
    return std::clamp(errora * .5f + derivativea * .0f + integrala * .0f,-127.0f,127.0f);
}
void drive(float goal, float power = 100, float timeout = 5000,float tolerance = .5) {
    float error = goal,prevError = goal,derivative{}, integral{},time{},current{},complete{};
    tare();
    while(1) {
        current = getAvInches();
        error = goal - current;
        if (fabs(error) < 5) {
            integral += error;
        }
        else if (fabs(error) < .5 || fabs(error) > 5) integral = 0;
    
    derivative = -(prevError - error)/20;
    float move = error * dkP + integral * dkI + derivative  * dkD;
    rmg.move(move * (power/100));
    lmg.move(move * (power/100));
    if (fabs(error) < tolerance) {
        complete ++;
    }
    else complete = 0;
    if (complete == 10) break;
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    lift.move(liftPIDDR(liftgoal));
    pros::delay(20);
    }
    rmg.move(0);
    lmg.move(0);
}
float turnplace{};
void turn(float goal, float power = 100, float timeout = 3000, float tolerance = .5) {
    float error = goal,prevError = goal,derivative{}, integral{},time{},current{},complete{};
    tare();
    while(1) {
        current = ethan.get_heading();
        error = wrapAngle(goal - current);
        if (fabs(error) < 5) {
            integral += error;
        }
        else if (fabs(error) < .5 || fabs(error) > 5) integral = 0;
    
    derivative = -(prevError - error);
    float move = error * turnkP + integral * turnkI + derivative  * turnkD;
    rmg.move(-move * (power/100));
    lmg.move(move * (power/100));
    if (fabs(error) < tolerance) {
        complete ++;
    }
    else complete = 0;
    if (paul.get_distance() < 400) {
        turnplace = ethan.get_heading();
    }
    if (complete == 5) break;
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    pros::lcd::print(0,"angle %.2f", ethan.get_heading());
    pros::lcd::print(1,"error %.2f",error);
    lift.move(liftPIDDR(liftgoal));
    pros::delay(20);
    }
    rmg.move(0);
    lmg.move(0);
}
void driveUntil(float goal, float power = 100, float timeout = 5000,float tolerance = .5) {
    float error = goal,prevError = goal,derivative{}, integral{},time{},current{};
    tare();
    while(1) {
        current = getAvInches();
        error = goal - current;
        if (fabs(error) < 5) {
            integral += error;
        }
        else if (fabs(error) < .5 || fabs(error) > 5) integral = 0;
    
    derivative = -(prevError - error);
    float move = error * dkP + integral * dkI + derivative  * dkD;
    rmg.move(move * (power/100));
    lmg.move(move * (power/100));
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    lift.move(liftPIDDR(liftgoal));
    pros::delay(20);
    }
    rmg.move(0);
    lmg.move(0);
}
void driveToHeading(float dis, float angle, float power = 100, float timeout = 6000) {
    tare();
    float dcurrent = getAvInches(),tcurrent = ethan.get_heading(),derror = dis, terror = angle,dprevError = dis, tprevError = angle,dderivative{}, tderivative{},dintegral{},tintegral{},time{},complete{};
    while(1) {
        dcurrent = getAvInches();
        tcurrent = ethan.get_heading();
        derror = dis - dcurrent;
        if (derror < 5) dintegral += derror;
        if (derror > 5 || dintegral < .5) dintegral = 0;
        dderivative = dprevError - derror;
        float drive = derror * 13 + dintegral * 0 + dderivative * 1.2;
        terror = wrapAngle(angle - tcurrent);
        if (terror < 15) tintegral += terror;
        if (terror > 15 || terror < 3) tintegral = 0;
        tderivative = tprevError - terror;
        float turn = terror * .865 + tintegral * 0 + tderivative * .4;
        rmg.move((drive-turn)*(power/100));
        lmg.move((drive+turn)*(power/100));
        if (terror < 5 && derror < 2) complete ++;
        else complete = 0;
        if (complete == 20) break;
        time += 20;
        if (time >= timeout) break;
        tprevError = terror;
        dprevError = derror;
        lift.move(liftPIDDR(liftgoal));
        pros::delay(20);
    }
    lmg.move(0);
    rmg.move(0);
}
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
}

void disabled() {}

void competition_initialize() {}

void endonmatchload() {
    drive(12,100);
    turn(91,100,1300);
    driveToHeading(12,turnplace,100,2000);
    lift.move(-30);
    pros::delay(600);
    claw.toggle();
    pros::delay(600);/*
    drive(-6.25,100,1500);
    turn(20,100,1000);
    liftgoal = 0;
    drive(13,100,4000);
    while(1) {
        lmg.move(27);
        rmg.move(-27);
        roller.print(0,0,"dis, %.2f",paul.get_distance());
        if (paul.get_distance() < 550) {
            break;
        }
    }
    
    liftgoal = 0;
    rmg.move(0);
    lmg.move(0);
    drive(6,50,1000);
    claw.toggle();
    pros::delay(200);
    liftgoal = 50;
    turn(160,100,2000);
    driveToHeading(10, turnplace);
    lift.move(-20);
    pros::delay(1000);
    claw.toggle();*/
}
void eomltranslated() {
    /*drive(4,400);
    drive(-4,400);
    drive(4,400);
    drive(-4,400);*/
    rob.setPose(0,0,0);
    rob.arcade(30,22);
    pros::delay(1100);
    rob.turnToHeading(90,400);
    roller.print(0,0,"x %.2f,%.2f",rob.getPose().x,rob.getPose().y);
    lift.move(-30);
    pros::delay(600);
    pros::delay(600);
    liftgoal = 0;
    rob.moveToPoint(8,18,3000,{.forwards = false},false);
    rob.moveToPoint(14,22,9000);
    rob.turnToHeading(90,1000,{.maxSpeed = 35},true);
    while(rob.isInMotion()) {
        if (paul.get_distance() < 400) {
            rob.cancelMotion();
        }
    }
}
void autonomous() { //use 35 for turning to look for something
    eomltranslated();
}
float errorb{},integralb{},derivativeb{},prevErrorb{};
float antitip() {
	if (fabs(integralb) < 20) {
	errorb = -ethan.get_roll();
	integralb += errorb;
	}
	else integralb = 0;
	derivativeb = errorb-prevErrorb;
	if (fabs(errorb) > 10) {
	return errorb * 6 + integralb * 0 + derivativeb * 0;
	}
	else return 0;
}

float curve(float input, float expo = 1.88) {
    float y = std::copysign(1,input);
    return std::pow(fabs(input/127.0f),expo) * y * 127.0f;
}
void opcontrol() {
float place{};
lift.set_brake_mode(pros::MotorBrake::hold);
rob.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while(1) {
		if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.toggle();
        }
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            tilt.toggle();
        }
        if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && claw.is_extended()) {
            lift.move(127);
        }
        else if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !claw.is_extended()) {
            lift.move(100);
        }
        if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && claw.is_extended()) {
            lift.move(-50);
        }
        else if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(-30);
        }
        if (roller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1) || roller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)) {
            place = lift.get_position();
            integrala = 0;
        }
        if (!roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(liftPIDDR(place));
        }
        if ((fabs(ethan.get_roll()) > 10)) {
            lift.move(-60);
        }
        pros::lcd::print(0, "X: %f", rob.getPose().x);
        pros::lcd::print(1, "Y: %f", rob.getPose().y);
        pros::lcd::print(2, "Theta: %f", rob.getPose().theta);
        pros::delay(20);
	rmg.move(curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) + antitip());
	lmg.move(curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) + antitip());
	pros::delay(20);
	}
}