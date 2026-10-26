#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
//github test
pros::Controller roller (pros::E_CONTROLLER_MASTER);
pros::MotorGroup lift({-21,2});
pros::MotorGroup rmg ({18,19,20});
pros::MotorGroup lmg ({-11,-12,-13});
pros::adi::Pneumatics claw ('A', false);
pros::adi::Pneumatics tilt ('B',false);
pros::Rotation hori (1);
pros::Rotation vert (15);
pros::Imu imu (14);
pros::Distance paul (10);

void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}
//ROBOT CONST
constexpr float ticks = 450;
constexpr float diameter = 3.125;
float horidis = sqrt(3.94);
float vertdis = sqrt(12.89);

//PID DRIVE CONST
constexpr float dkP = 13;
constexpr float dkI = .0f;
constexpr float dkD = 3;

//PID TURNING CONST
constexpr float turnkP = .865;
constexpr float turnkI = .1;
constexpr float turnkD = 1;

//POS
float x{},y{};
float liftgoal{};

float errorb{},prevErrorb{},integralb{},derivativeb{},currentb = lift.get_position();
float liftPID (float goal) {
    errorb = (goal*14) - currentb;
    currentb = lift.get_position();
    integralb += errorb;
    derivativeb = errorb - prevErrorb;
    prevErrorb = errorb;
    return std::clamp(errorb * .5f + derivativeb * .0f + integralb * .0f,-127.0f,127.0f);
}
float liftPIDDR (float goal) {
    errorb = (goal) - currentb;
    currentb = lift.get_position();
    integralb += errorb;
    derivativeb = errorb - prevErrorb;
    prevErrorb = errorb;
    return std::clamp(errorb * .5f + derivativeb * .0f + integralb * .0f,-127.0f,127.0f);
}
float vInches() { return vert.get_position() * (M_PI * 2) / -36000;}
float hInches() { return hori.get_position() * (M_PI * 2) / 36000;}

float getLeftInches() {
    return lmg.get_position()/ticks * (M_PI * diameter);
}

float getRightInches() {
    return rmg.get_position()/ticks * (M_PI * diameter);
}

float getAvInches() {
    return (getLeftInches() + getRightInches())/2;
}

void tare() {
    rmg.tare_position();
    lmg.tare_position();
}

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    imu.reset();
    pros::lcd::register_btn1_cb(on_center_button);
}

float wrapAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

float getHeading() {
    return wrapAngle(imu.get_heading());
}

float dtr(float angle) {
    return angle * (M_PI/180);
}

float rtd(float angle) {
    return angle * (180/M_PI);
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
    
    derivative = -(prevError - error);
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
    lift.move(liftPID(liftgoal));
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
        current = getHeading();
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
        turnplace = getHeading();
    }
    if (complete == 5) break;
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    pros::lcd::print(0,"angle %.2f", getHeading());
    pros::lcd::print(1,"error %.2f",error);
    lift.move(liftPID(liftgoal));
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
    lift.move(liftPID(liftgoal));
    pros::delay(20);
    }
    rmg.move(0);
    lmg.move(0);
}
void driveToHeading(float dis, float angle, float power = 100, float timeout = 6000) {
    tare();
    float dcurrent = getAvInches(),tcurrent = getHeading(),derror = dis, terror = angle,dprevError = dis, tprevError = angle,dderivative{}, tderivative{},dintegral{},tintegral{},time{},complete{};
    while(1) {
        dcurrent = getAvInches();
        tcurrent = getHeading();
        derror = dis - dcurrent;
        if (derror < 5) dintegral += derror;
        if (derror > 5 || dintegral < .5) dintegral = 0;
        dderivative = dprevError - derror;
        float drive = derror * dkP + dintegral * dkI + dderivative * dkD;
        terror = wrapAngle(angle - tcurrent);
        if (terror < 15) tintegral += terror;
        if (terror > 15 || tintegral < 3) tintegral = 0;
        tderivative = tprevError - terror;
        float turn = terror * turnkP + tintegral * turnkI + tderivative * turnkD;
        rmg.move((drive-turn)*(power/100));
        lmg.move((drive+turn)*(power/100));
        if (terror < 5 && derror < 2) complete ++;
        else complete = 0;
        if (complete == 20) break;
        time += 20;
        if (time >= timeout) break;
        tprevError = terror;
        dprevError = derror;
        lift.move(liftPID(liftgoal));
        pros::delay(20);
    }
    lmg.move(0);
    rmg.move(0);
}

float pVp{},pVh{},pTheta{};
void updateCoord() {
    float dv = vInches()-pVp,dh = hInches() - pVh,dtheta = wrapAngle(getHeading() - pTheta);
    float dX = -(dh - horidis * (dtr(dtheta))) * cos(dtr(getHeading())) - (dv - vertdis*(dtr(dtheta))) * sin(dtr(getHeading()));
    float dY = -(dv - vertdis * (dtr(dtheta))) * cos(dtr(getHeading())) + (dh - horidis*(dtr(dtheta))) * sin(dtr(getHeading()));
    x += dX;
    y += dY;
    pVp = vInches();
    pVh = hInches();
    pTheta = getHeading();
}
float errora = 0 - imu.get_roll(), currenta = imu.get_roll(),prevErrora = 0,integrala{},derivativea{};
float antitip() {
        errora = -imu.get_roll();
        currenta = imu.get_roll();
        integrala += errora;
        derivativea = errora - prevErrora;
        float output = errora * 6 + integrala * 0 + derivativea * 0;
        prevErrora = errora;
        if (fabs(errora) > 20) {
        return output;
        }
        else return 0;
}
void disabled() {}

void competition_initialize() {}

void leftSideAuto1point() {
    liftgoal = 20;
    drive(12);
    turn(-90);
    drive(12.5);
    lift.move(-20);
    pros::delay(1000);
    liftgoal = 0;
    claw.toggle();
    pros::delay(100);
    drive(-6);
    turn(205);
    drive(12);
    turn(-90);
    drive(16);
}

void rightSideAuto1point() {
    liftgoal = 20;
    drive(12);
    turn(90);
    drive(12.5);
    lift.move(-20);
    pros::delay(1000);
    liftgoal = 0;
    claw.toggle();
    pros::delay(100);
    drive(-6);
    turn(150);
    drive(12);
    turn(90);
    drive(16);
}
void toggleTwice() {
    drive(4,400);
    drive(-4,400);
    drive(4,400);
    drive(-4,400);

}
void toggle() {
    drive(4,400);
    drive(-4,400);
}
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

void leftendonmatchload() {
    liftgoal = 20;
    drive(12,100);
    turn(-88.5,100,1300);
    driveToHeading(12,turnplace,100,2000);
    lift.move(-30);
    pros::delay(600);
    claw.toggle();
    pros::delay(600);
    /*
    drive(-6.25,100,1500);
    turn(-20,100,1000);
    liftgoal = 0;
    drive(13,100,4000);
    while(1) {
        lmg.move(-27);
        rmg.move(27);
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
    turn(-160,100,2000);
    driveToHeading(10, turnplace);
    lift.move(-20);
    pros::delay(1000);
    claw.toggle();
*/
}
void autonomous() {
    lift.move(60);
    pros::delay(200);
    tilt.toggle();
    lift.move(-40);
    liftgoal = 0;
    pros::delay(200);
    endonmatchload();
}

float curve(float input, float expo = 1.88) {
    float y = std::copysign(1,input);
    return std::pow(fabs(input/127.0f),expo) * y * 127.0f;
}
int tim{};
bool locked = false;
void opcontrol() {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    float place{};
    liftgoal = 50;
    while (1) {
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.toggle();
        }
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            tilt.toggle();
        }
        if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && claw.is_extended()) { ///1400 is the point whwre the motor is at the highest point
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
        }
        if (!roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(liftPIDDR(place));
        }
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            locked = !locked;
        }
        if ((fabs(imu.get_roll()) > 25)) {
            lift.move(-60);
        }
        if (!locked) {
            lmg.set_brake_mode(pros::MotorBrake::coast);
            rmg.set_brake_mode(pros::MotorBrake::coast);
            rmg.move((curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),2)) + antitip());
            lmg.move((curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),2)) + antitip());
        }
        if (locked) {
            rmg.set_brake_mode(pros::MotorBrake::brake);
            lmg.set_brake_mode(pros::MotorBrake::brake);
            rmg.move(0);
            lmg.move(0);
        }
        //updateCoord();
        pros::lcd::print(1,"x %.2f", x);
        pros::lcd::print(0,"y %.2f",y);
        pros::delay(20);
    }
        
}