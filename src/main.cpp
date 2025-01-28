#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <cmath>
#include <iostream>
#include <ostream>
#include <string>

bool isFlipping = false;

bool isRaised = false;
bool mogoOn = false;
bool doinkerOn = false;


bool canRaise = true;
bool canMogo = true;
bool canDoinker = true;

pros::MotorGroup left_motor_group({20, -18, -17}, pros::v5::MotorGears::blue);

pros::MotorGroup right_motor_group({-19, 16, 8}, pros::v5::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.32, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 
);

pros::Motor intake(6, pros::v5::MotorGears::blue);

pros::Motor flipper(7, pros::v5::MotorGears::red);

pros::adi::DigitalOut mogo ('C');

pros::adi::DigitalOut doinker ('E');

pros::adi::DigitalOut arm ('D');

pros::adi::Button limit_switch('F');

pros::adi::Ultrasonic ultrasonic ('G', 'H');

pros::Vision vision(15);

pros::Imu imu(1);

pros::Rotation rotation(-11);

lemlib::TrackingWheel horizontal_tracking_wheel(&rotation, lemlib::Omniwheel::NEW_2, 2.25);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              30, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              30, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);

int autonState = 0;
bool buttonUnpressed = true;

int numRings = 0;
bool hasSecond = false;

std::string stack[2] = {"", ""};

void doFlip(){
    isFlipping = true;
    if(((autonState == 0 || autonState == 1 || autonState == 2) && stack[0] == "Red") 
    || ((autonState == 3 || autonState == 4 || autonState == 5) && stack[0] == "Blue")){
        flipper.move_absolute(-750, 150);
    } else {
        flipper.move_absolute(-1060, 200);
    }
    pros::delay(600);
    flipper.move_absolute(0, 100);
    pros::delay(800);
    isFlipping = false;
}

void toggleArm(){
    canRaise = false;
    arm.set_value(!isRaised);
    isRaised = !isRaised;
    pros::delay(1000);
    canRaise = true;
}

void raiseMacro(){
    canRaise = false;
    if(!isRaised){
        arm.set_value(!isRaised);
        isRaised = !isRaised;
    }
    isFlipping = true;
    flipper.move_absolute(-1700, 200);
    pros::delay(1000);
    isFlipping = false;
    canRaise = true;
}

void toggleMogo(){
    canMogo = false;
    mogo.set_value(!mogoOn);
    mogoOn = !mogoOn;
    if(!mogoOn){
        controller.print(0, 0, "%s", "Mogo on ");
    } else {
        controller.print(0, 0, "%s", "Mogo off");
    }

    pros::delay(250);
    canMogo = true;
}

void toggleDoinker(){
    canDoinker = false;
    doinker.set_value(!doinkerOn);
    doinkerOn = !doinkerOn;
    pros::delay(1000);
    canDoinker = true;
}

void shaky(){
    isFlipping = true;
    flipper.move_velocity(-35);
    pros::delay(250);
    flipper.move_absolute(0, 55);
    pros::delay(250);
    isFlipping = false;
}

void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "Auton: %d", autonState);
    chassis.calibrate();
    flipper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task auton_switch([&](){
        while(true)
        {
            if(limit_switch.get_value() == 1 && buttonUnpressed)
            {
                autonState++;
                if(autonState == 6) autonState = 0;
                pros::lcd::print(0, "Auton: %d", autonState);
                buttonUnpressed = false;
            }
            if(limit_switch.get_value() != 1) buttonUnpressed = true;

            // pros::delay(7);
            controller.print(0, 0, "Angle: %.1f", imu.get_heading());
            std::cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " Theta: " << chassis.getPose().theta << std::endl;
            pros::delay(50);
        }
    });
}

// void disabled() {}

ASSET(chunky_txt)

void autonomous() {
    chassis.setPose(0,0,0);
    chassis.moveToPose(0,24,0,10000);
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.moveToPoint(17.424, 32.098, 5000);
    // chassis.moveToPoint(15.721, 45.199, 5000);
    // chassis.moveToPoint(15.59, 31.312, 5000);
    // chassis.moveToPoint(1.31, 31.443, 5000);
    // chassis.moveToPoint(-36.552, 3.144, 5000);
    // chassis.moveToPoint(-31.443, -3.799, 5000);
    // chassis.moveToPoint(-31.443, -8.123, 5000);
    // chassis.moveToPoint(-78.869, 31.836, 5000);
    // chassis.moveToPoint(-82.275, 48.081, 5000);
    // chassis.moveToPoint(-64.457, 31.312, 5000);
    // chassis.moveToPoint(-46.116, 37.862, 5000);

}

void update_stack(){
    while(true){
        pros::vision_object_s_t object_arr[3];

        int y1 = 0;
        int y2 = 0;
        std::string color1 = "";
        std::string color2 = "";

        vision.read_by_size(0, 3, object_arr);

        for(int i = 0; i < 3; i++){
            if(object_arr[i].height > 150 && (object_arr[i].signature == 1 || object_arr[i].signature == 2)){
                if(y1 != 0){
                    y2 = object_arr[i].y_middle_coord;
                    color2 = (object_arr[i].signature == 1) ? "Red" : "Blue";
                } else {
                    y1 = object_arr[i].y_middle_coord;
                    color1 = (object_arr[i].signature == 1) ? "Red" : "Blue";
                }
            }
            if(y2 < y1){
                stack[1] = color2;
                stack[0] = color1;
            } else {
                stack[1] = color1;
                stack[0] = color2;
            }
        }

        std::cout << "Bottom: " +  stack[0] << " Top: " + stack[1] << std::endl;

        pros::delay(200);
    }
}

void opcontrol() {
    // loop forever 
    pros::Task vision_task(update_stack);
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        if(leftY < 0){
            chassis.arcade(leftY*.8, rightX * .7);
        } else if (leftY > 0){
            chassis.arcade(leftY, rightX * .7);
        }
        else {
            chassis.arcade(leftY, rightX);
        }

        hasSecond = ultrasonic.get_value() > 50;

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && flipper.get_position() > -50 && (!(stack[0] != "" && stack[1] != "") || controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))){
            intake.move_velocity(-600);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake.move_velocity(550);
        }
        else {
            intake.move_velocity(0);
        }

        if(!isFlipping){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
                flipper.move(-127);
            } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                flipper.move(50);
            } else {
                flipper.brake();
            }
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && canRaise){
            pros::Task raise_task(toggleArm);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && canRaise){
            pros::Task raise_macro(raiseMacro);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && canMogo){
            pros::Task mogo_task(toggleMogo);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && canDoinker){
            pros::Task doinker_task(toggleDoinker);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !isFlipping){
            pros::Task flip_task(doFlip);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && !isFlipping){
            pros::Task shake_task(shaky);
        }

        // delay to save resources
        pros::delay(2);
    }
}