#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
//github test
pros::Controller roller (pros::E_CONTROLLER_MASTER);
pros::MotorGroup lift({-21,2});
pros::MotorGroup RMG ({18,19,20});
pros::MotorGroup LMG ({-11,-12,-13});
pros::adi::Pneumatics claw ('A', false);
pros::adi::Pneumatics tilt ('B',false);
pros::Rotation hori (16);
pros::Rotation vert (17);
pros::Imu imu (15);

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
constexpr float dkI = 0;
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

float vInches() { return vert.get_position() * (M_PI * 2) / -36000;}
float hInches() { return hori.get_position() * (M_PI * 2) / 36000;}

float getLeftInches() {
    return LMG.get_position()/ticks * (M_PI * diameter);
}

float getRightInches() {
    return RMG.get_position()/ticks * (M_PI * diameter);
}

float getAvInches() {
    return (getLeftInches() + getRightInches())/2;
}

void tare() {
    RMG.tare_position();
    LMG.tare_position();
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
    RMG.move(move * (power/100));
    LMG.move(move * (power/100));
    if (fabs(error) < tolerance) {
        complete ++;
    }
    else complete = 0;
    if (complete == 10) break;
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    roller.print(2,0,"dis %.2f",getAvInches());
    roller.print(3,0,"error %.2f",error);
    lift.move(liftPID(liftgoal));
    pros::delay(20);
    }
    RMG.move(0);
    LMG.move(0);
}

void turn(float goal, float power = 100, float timeout = 3000, float tolerance = 1) {
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
    RMG.move(-move * (power/100));
    LMG.move(move * (power/100));
    if (fabs(error) < tolerance) {
        complete ++;
    }
    else complete = 0;
    if (complete == 5) break;
    time += 20;
    if (time >= timeout) break;
    prevError = error;
    pros::lcd::print(0,"angle %.2f", getHeading());
    pros::lcd::print(1,"error %.2f",error);
    lift.move(liftPID(liftgoal));
    pros::delay(20);
    }
    RMG.move(0);
    LMG.move(0);
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
        terror = angle - tcurrent;
        if (terror < 15) tintegral += terror;
        if (terror > 15 || tintegral < 3) tintegral = 0;
        tderivative = tprevError - terror;
        float turn = terror * turnkP + tintegral * turnkI + tderivative * turnkD;
        RMG.move((drive-turn)*(power/100));
        LMG.move((drive+turn)*(power/100));
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
    LMG.move(0);
    RMG.move(0);
}

float pVp{},pVh{},pTheta{};
void updateCoord() {
    float dv = vInches()-pVp,dh = hInches() - pVh,dtheta = wrapAngle(getHeading() - pTheta);
    float dX = (dh - horidis * (dtr(dtheta))) * cos(dtr(getHeading())) + (dv - vertdis*(dtr(dtheta))) * sin(dtr(getHeading()));
    float dY = (dv - vertdis * (dtr(dtheta))) * cos(dtr(getHeading())) + (dh - horidis*(dtr(dtheta))) * sin(dtr(getHeading()));
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
        float output = errora * 4 + integrala * 0 + derivativea * 0;
        prevErrora = errora;
        if (fabs(errora) < 5) {
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
    turn(210);
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

void endonmatchload() {
    liftgoal = 20;
    drive(12,100,9000);
    turn(90,100,1300);
    drive(8.5,100,5000,.25);
    lift.move(-30);
    pros::delay(600);
    claw.toggle();
    pros::delay(600);
    drive(-6,150,1500);
    turn(28,100,2000,.125);
    liftgoal = 0;
    drive(10,100,6000);
    drive(9.5,30,3000);
    claw.toggle();
    pros::delay(400);
    liftgoal = 50;
    turn(170);
    drive(12,50,3000,.5);
    lift.move(-20);
    claw.toggle();
    pros::delay(400);
    drive(-2,300,700);
    turn(150);
    drive(20);
}

void autonomous() {
    endonmatchload();
}

float curve(float input, float expo = 1.88) {
    float y = std::copysign(1,input);
    return std::pow(fabs(input/127.0f),expo) * y * 127.0f;
}
int tim{};

void opcontrol() {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    float place{};
    while (1) {
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.toggle();
        }
        if (roller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            //tilt.toggle();
        }
        if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (lift.get_position() <=1400) && claw.is_extended()) { ///1400 is the point whwre the motor is at the highest point
            lift.move(100);
        }
        else if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(127);
        }
        if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && claw.is_extended()) {
            lift.move(-20);
        }
        else if (roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(-50);
        }
        if (roller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1) || roller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)) {
            place = lift.get_position();
        }
        if (!roller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !roller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(liftPID(place));
        }
        RMG.move((curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y))+antitip()));
        LMG.move((curve(roller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))+antitip()));
        //updateCoord();
        pros::lcd::print(1,"x %.2f", x);
        pros::lcd::print(0,"y %.2f",y);
        roller.print(0,0,"pos %.2f", antitip());
        pros::delay(20);
    }
        
}