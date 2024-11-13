#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"

bool isFlipping = false;

// pros::MotorGroup left_motor_group({-1, -3, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

// pros::MotorGroup right_motor_group({2, 4, 5}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

// lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
//                               &right_motor_group, // right motor group
//                         12.75, // 10 inch track width
//                               lemlib::Omniwheel::NEW_325, // using new 4" omnis
//                               400, // drivetrain rpm is 360
//                               2 
// );

pros::Motor intake(6, pros::v5::MotorGears::blue);

pros::Motor flipper(7, pros::v5::MotorGears::red);

// pros::Imu imu(19);

// lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             nullptr, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

// // lateral PID controller
// lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               3, // derivative gain (kD)
//                                               3, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               20 // maximum acceleration (slew)
// );

// // angular PID controller
// lemlib::ControllerSettings angular_controller(2.2, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               14, // derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in inches
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in inches
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// // input curve for throttle input during driver control
// lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

// // create the chassis
// lemlib::Chassis chassis(drivetrain,
//                         lateral_controller,
//                         angular_controller,
//                         sensors,
//                         &throttle_curve, 
//                         &steer_curve
// );

// void initialize() {
//     // pros::lcd::initialize(); // initialize brain screen
//     chassis.calibrate(); // calibrate sensors
//     // print position to brain screen
//     pros::Task screen_task([&]() {
//         while (true) {
//             // print robot location to the brain screen
//             pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
//             pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
//             pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
//             // delay to save resources
//             pros::delay(20);
//         }
//     });
// }

// void disabled() {}

// void competition_initialize() {}

// void autonomous() {
//     // set position to x:0, y:0, heading:0
//     chassis.setPose(0, 0, 0);
//     // turn to face heading 90 with a very long timeout
//     chassis.turnToHeading(120, 100000);
// }

void doFlip(){
    isFlipping = true;
    flipper.move_absolute(-1060, 200);
    pros::delay(500);
    flipper.move_absolute(0, 30);
    pros::delay(1000);
    isFlipping = false;
}

void opcontrol() {
    // loop forever 
    pros::lcd::initialize(); // initialize brain screen
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if(isFlipping){
            pros::lcd::print(0, "%s", "True"); // x
        } else {
            pros::lcd::print(0, "%s", "False"); // x
        }


        // move the robot
        // chassis.arcade(leftY, rightX*.7, false, 0.95);
        // chassis.curvature(leftY, rightX);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && flipper.get_position() > -50){
            intake.move(-127);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move(80);
        }
        else {
            intake.move(0);
        }

        if(!isFlipping){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                flipper.move(-30);
            } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
                flipper.move(30);
            } else {
                flipper.brake();
            }
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !isFlipping){
            pros::Task flip_task(doFlip);
        }

        // delay to save resources
        pros::delay(2);
    }
}