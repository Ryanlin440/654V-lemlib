#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
void descoreMacro(bool forward);
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
		pros::lcd::set_text(2, "Oma gawd!");
	} else {
		pros::lcd::clear_line(2);
	}
}

pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::MotorGroup left_motor_group({-2, -3, -4}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({18, 19, 20}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              12,                    // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              800, // drivetrain rpm is 800
                              8    // horizontal drift is 2 (for now),
                              
);

// create an imu on port 8
pros::Imu imu(8);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(-1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,0);
pros::Rotation horizontal_encoder(12);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);

pros::Motor conveyorMotor (9, pros::MotorGears::blue);
pros::Motor topMotor(10, pros::MotorGears::blue);

pros::adi::DigitalOut scraper('a',LOW);
pros::adi::DigitalOut middleGoal('b', HIGH);
pros::adi::DigitalOut leftDescore('c',LOW);
bool scraperBool, middleGoalBool, leftDescoreBool;

bool forwardOritentiation = true;

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0.12, // integral gain (kI)-.006 over .41
                                              10, // derivative gain (kD) 
                                              1.8, // anti windup 1.8
                                              1, // small error range, in inches  1
                                              100, // small error range timeout, in milliseconds 100
                                              3, // large error range, in inches  3
                                              500, // large error range timeout, in milliseconds 500
                                              20 // maximum acceleration (slew)20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.5,   // proportional gain (kP)
                       0.08,   // integral gain (kI) .03 too little
                       25.0402,  // derivative gain (kD)//10.7 w/ ki  14.5
                       25,   // anti windup  3.25184
                        1,   // small error range, in degrees  1
                       100, // small error range timeout, in milliseconds  100
                       3,   // large error range, in degrees  3
                       500, // large error range timeout, in milliseconds  500
                       0    // maximum acceleration (slew)  0
);

lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain      
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain    
);
// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve,
                        &steer_curve
);



int selectedAuton = 0;
bool autonSelected = false;
const char* autonNames[] = {"Middle Control", "All Long Goal", "Four Ball Rush", "Right Side", "SAWP", "Skills", "User"};
void autonSelector() {
    controller.clear();

    while (!autonSelected) {
        controller.print(2, 0, "Auton: %-8s", autonNames[selectedAuton]);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton--;
            if (selectedAuton < 0) selectedAuton = 0;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton++;
            if (selectedAuton > 6) selectedAuton = 6;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            autonSelected = true;
        }

        pros::delay(20);
    }

    
    // controller.print(2, 0, "%-10s",  );
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
    scraperBool = false;
    middleGoalBool = HIGH;
    leftDescoreBool = false;
    pros::Task screenTask([&]() {
      while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
      }
    });

    autonSelector();//remove this aswell
    autonomous();//⚠️ Remove this before competitions.
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



void competition_initialize() {
    // 0 is middle control
    // 1 is all long goal
    // 2 is four ball rush
    // 3 is right side AWP
    // 4 is SAWP
    // 5 is skills
    // 6 is user testing or opcontrol

    // selectedAuton = 0;  //change this everytime
    //autonSelector();
}
void tuningLateralPD() {
        chassis.turnToPoint(0, 24, 2000);
        chassis.moveToPoint(0, 24, 2000);
        pros::delay(2000);
        chassis.turnToPoint(0, 48, 2000);
        chassis.moveToPoint(0, 48, 2000);
        pros::delay(2000);
        chassis.moveToPose(0, 0, 0, 2000, {.forwards = false});//.earlyExitRange = 0}
}
void tuningLateralI() {
        chassis.turnToPoint(0, 24, 5000);
        chassis.moveToPoint(0, 24, 5000);
        pros::delay(6000);

        chassis.turnToPoint(0, 48, 5000);
        chassis.moveToPoint(0, 48, 5000);
        pros::delay(6000);

        chassis.moveToPose(0, 0, 0, 4000, {.forwards = false});//.earlyExitRange = 0}
  
}

void tuningAngularPD() {
        chassis.turnToHeading(90, 5000);
        chassis.turnToHeading(180, 5000);
        chassis.turnToHeading(0, 5000);

}
void tuningAngularI() {
        chassis.turnToHeading(90, 5000);
        pros::delay(5000);
        chassis.turnToHeading(180, 5000);
        pros::delay(5000);
        chassis.turnToHeading(0, 5000);
}




void leftMiddleControl() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-5, 46, 2800, {.maxSpeed = 50});
    pros::delay(500);
    scraper.set_value(LOW);

    // Middle Scoring
    chassis.moveToPose(-7, 14, 315, 5000, {.forwards = false, .lead = .3});
    pros::delay(900);
    topMotor.move(50);
    pros::delay(1000); // make longer if want to score in middle more
    conveyorMotor.move(0);
    topMotor.move(0);
    
    // Going in front of loader
    chassis.turnToPoint(-45, 51, 3000);
    chassis.moveToPoint(-45, 51, 5000);
    scraper.set_value(HIGH);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-47, 47, 270);

    // Going into loader 
    chassis.moveToPoint(-55, 47, 1000, {.minSpeed = 60});//put to -60 once done
	conveyorMotor.move(127);
    pros::delay(1000);

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, 47, 2000, {.forwards = false});
    pros::delay(700);
    topMotor.move(127);
    pros::delay(2000);
    
    // Descore
    descoreMacro(true);
    pros::delay(500);

}
void leftAllLongGoal() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-5, 46, 2800, {.maxSpeed = 50});
    pros::delay(500);
    scraper.set_value(LOW);

    // Going to position to go in front of loader
    chassis.moveToPose(-22.22, 22.22, 315, 5000, {.forwards = false, .lead = .3});
    conveyorMotor.move(0);
    
    // Getting in front of loader
    chassis.turnToPoint(-45, 51, 3000); // prob need to change
    chassis.moveToPoint(-45, 51, 5000); // prob need to change
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-47, 47, 270);

    // Going into loader
    scraper.set_value(HIGH);
    chassis.moveToPoint(-55, 47, 1000, {.minSpeed = 60});//put to -60 once done
	conveyorMotor.move(127);
    pros::delay(1500); // time how much seconds

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, 47, 2000, {.forwards = false});
    pros::delay(700);
    topMotor.move(127);
    scraper.set_value(LOW);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Get rid of during competition
    pros::delay(5000); 
    conveyorMotor.move(0);
    topMotor.move(0);
}
void leftFourBallRush() {
    // Three balls 
    chassis.moveToPose(-22.22, 22.22, 30, 2000, {.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    
    // Going in front of long goal
    // chassis.moveToPose(-27, 47, 0, 3000, {.forwards = false, .lead = .01}); Smoother if works
    chassis.turnToPoint(-40, 48, 3000, {.forwards = false}); // prob need to change
    chassis.moveToPoint(-40, 48, 3000, {.forwards = false}); // prob need to change
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-40, 48, 270);
    scraper.set_value(LOW);

    //Scoring in long goal
    chassis.moveToPoint(-25, 47, 2000, {.forwards = false});
    pros::delay(300);
    topMotor.move(127);
    pros::delay(2000);
    descoreMacro(true);
}
void rightSimpleAWP() {
chassis.moveToPose(
      -22.22, -22.22, 150, 2000,
      {.lead = .3, .minSpeed = 50}); //{.horizontalDrift = 8, .lead = 0.3}
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, -43, 3000,  {.maxSpeed=50});//-5, 46 for one
    pros::delay(200);
    scraper.set_value(LOW);
    chassis.moveToPoint(-17, -22, 2000, {.forwards = false});
    pros::delay(7000);
    chassis.turnToPoint(-5, -11, 2000);
    pros::delay(7000);

    chassis.moveToPoint(-5, -11, 2000);
    // chassis.moveToPose(-8, -14, 315, 5000, {.forwards = false, .lead = .3});


    pros::delay(500);
    topMotor.move(-50);
    conveyorMotor.move(-127);
    chassis.turnToPoint(-45, -50, 3000, {.forwards = false});//////
    pros::delay(200);
    conveyorMotor.move(0);
    topMotor.move(0);
    chassis.moveToPoint(-45, -50, 5000, {.forwards = false});/////
    scraper.set_value(HIGH);
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-47, -47, 270);
    conveyorMotor.move(127);
    chassis.moveToPoint(-25, -47, 2000, {.forwards = false});
    pros::delay(800);
    topMotor.move(127);
    pros::delay(2000);
    topMotor.move(0);
    chassis.moveToPoint(-60, -47, 1000, {.minSpeed = 50}, false);
    pros::delay(1000);
    chassis.turnToPoint(-25, -47, 2000, {.forwards = false});
    chassis.moveToPoint(-25, -47, 2000, {.forwards = false});
    pros::delay(800);
    topMotor.move(127);
    pros::delay(2000);
  
}
void SAWP() { // place only on right side
    //First loader
    chassis.moveToPoint(-45, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5}); //Position first 
    scraper.set_value(HIGH);
    chassis.moveToPose(-58, -46, 270, 400, {.lead = .1, .minSpeed = 80});//The scraper boomerang
    chassis.turnToHeading(270, 100);
    chassis.turnToPoint(-60, -47, 200);
    chassis.moveToPoint(-60, -47, 500, {.minSpeed = 40});
    conveyorMotor.move(127);
    pros::delay(250);

    //Scoring in first goal
    chassis.turnToPoint(-27, -48, 1000, {.forwards = false}); // not needed if straight
    chassis.moveToPoint(-27, -48,4000, {.forwards = false}); //Long goal - X smaller to get closer
    pros::delay(600);
    topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1000);
    topMotor.move(0);

    //First 3 balls
    chassis.moveToPoint(-27, -26, 5000, {.minSpeed=40, .earlyExitRange=1});
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(500);
    chassis.turnToPoint(-24, 26, 2000, {}, false);
    scraper.set_value(LOW);

    // Second 3 balls

    chassis.moveToPoint(-24, 24, 2000, {});
    pros::delay(800);
    scraper.set_value(HIGH);
    

    //// Middle scoring par
    chassis.turnToPoint(-14,14, 1000, {.forwards = false});
    chassis.moveToPoint(-14, 14, 5000,  {.forwards = false});


    pros::delay(100);
    topMotor.move(50);
    pros::delay(1000);
    // chassis.turnToPoint(-47, 47, 2000);
    chassis.moveToPoint(-47, 53, 5000);
    topMotor.move(0);
    // pros::delay(7000);
    // return;
    // // Second loader
    // // scraper.set_value(HIGH);
    chassis.turnToPoint(-60, 53, 2000);
    chassis.moveToPoint(-60, 53, 3000, {.minSpeed=40});
    pros::delay(1000);

    // Second goal scoring
    chassis.turnToPoint(-30, 53, 2000, {.forwards = false});
    chassis.moveToPoint(-30, 53, 5000, {.forwards = false, .minSpeed=40});
    pros::delay(500);
    topMotor.move(127);
    pros::delay(2000);
    // topMotor.move(127);
}
void skills() {
    chassis.setPose(-47, 15, 0);

    // chassis.moveTo(-47, 15, 5000); starting position
    chassis.turnToPoint(-47, 52, 5000);
    chassis.moveToPoint(-47, 52, 5000);//get in front of loader
    scraper.set_value(HIGH);

    chassis.turnToPoint(-60, 52, 5000);//get balls in loader
    chassis.moveToPoint(-60, 52, 5000);
    conveyorMotor.move(127);
    pros::delay(2000);

    // chassis.turnToPoint(-47, 47, 5000, {.forwards = false});//back up a bit
    // chassis.moveToPoint(-47, 47, 5000, {.forwards = false});
    // scraper.set_value(LOW);
    // conveyorMotor.move(0);

    // chassis.turnToPoint(-36, 59, 5000);//get in position to go across
    // chassis.moveToPoint(-36, 59, 5000);
    
    // chassis.turnToPoint(36, 59, 5000);//get to other side
    // chassis.moveToPoint(36, 59, 5000);

    // chassis.turnToPoint(47, 47, 5000);//get in front of goal
    // chassis.moveToPoint(47, 47, 5000);

    // chassis.turnToPoint(32, 47, 5000, {.forwards = false});//score first goal
    // chassis.moveToPoint(32, 47, 5000, {.forwards = false});
    // // conveyorMotor.move(127);
    // // topMotor.move(127);
    // // scraper.set_value(HIGH);
    // pros::delay(2000);

    // chassis.turnToPoint(56, 47, 5000);//get balls in loader
    // chassis.moveToPoint(56, 47, 5000);
    // // topMotor.move(0);
    // pros::delay(2000);

    // chassis.turnToPoint(32, 47, 5000, {.forwards = false});//score more
    // chassis.moveToPoint(32, 47, 5000, {.forwards = false});
    // pros::delay(1000);
    // // topMotor.move(127);
    // pros::delay(2000);

    // chassis.turnToPoint(40, 47, 5000);//get in position to go across
    // chassis.moveToPoint(40, 47, 5000);
    // conveyorMotor.move(0);
    // topMotor.move(0);

    //------------------------------------------------------second half----------------------------------------------------//
    // chassis.turnToPoint(40, -47, 5000);//get to other side
    // chassis.moveToPoint(40, -47, 5000);

    // chassis.turnToPoint(56, -47, 5000);//get loader
    // chassis.moveToPoint(56, -47, 5000);
    // conveyorMotor.move(127);
    // pros::delay(2000);

    // chassis.turnToPoint(40, -47, 5000, {.forwards = false});//get in position to go to other side
    // chassis.moveToPoint(40, -47, 5000, {.forwards = false});
    // conveyorMotor.move(0);
    // scraper.set_value(LOW);

    // chassis.turnToPoint(36, -59, 5000);//get ready to cross
    // chassis.moveToPoint(36, -59, 5000);

    // chassis.turnToPoint(-36, -59, 5000);//get to other side
    // chassis.moveToPoint(-36, -59, 5000);

    // chassis.turnToPoint(-47, -47, 5000);//get in front of goal
    // chassis.moveToPoint(-47, -47, 5000);

    // chassis.turnToPoint(-32, -47, 5000, {.forwards = false});//score first goal
    // chassis.moveToPoint(-32, -47, 5000, {.forwards = false});
    // conveyorMotor.move(127);
    // topMotor.move(127);
    // scraper.set_value(HIGH);
    // pros::delay(2000);

    // chassis.turnToPoint(-56, -47, 5000);//get balls in loader
    // chassis.moveToPoint(-56, -47, 5000);
    // topMotor.move(0);
    // pros::delay(2000);
    
    // chassis.turnToPoint(-32, -47, 5000, {.forwards = false});//score more
    // chassis.moveToPoint(-32, -47, 5000, {.forwards = false});
    // pros::delay(1000);
    // topMotor.move(127);
    // scraper.set_value(LOW);
    // pros::delay(2000);

    // chassis.turnToPoint(-58, -23, 5000);//get readdy to park
    // chassis.moveToPoint(-58, -23, 5000);//get readdy to park

    // chassis.turnToPoint(-62.5, 0, 5000);//get readdy to park
    // chassis.moveToPoint(-62.5, 0, 5000);//parrkkkkkk
    // pros::delay(500);
    // scraper.set_value(HIGH);

}

void descoreMacro(bool forward) {
    forwardOritentiation = false;
    scraper.set_value(LOW);
    leftDescore.set_value(LOW);
	conveyorMotor.move(0);
    topMotor.move(0);

    chassis.setPose(0,0,0);
    chassis.moveToPose(12.3, 14, 3, 1000, {.lead = .01, .minSpeed = 50});
    // pros::delay(100);
    chassis.turnToHeading(3, 2000);
    if(forward){
        chassis.moveToPoint(10, -10, 2000, {.forwards = false, .minSpeed=30});
        pros::delay(3000);
        left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
}
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
    /*
    // conveyorMotor.move(127);
    // pros::delay(100000);
    // chassis.setPose(-61.3, -16.8, 90);
    //             chassis.setPose(-47, 15, 0);
    // pros::delay(1000);
    // skills();

    // tuningLateralI();

    // chassis.setPose(-61.3, 16.8, 90);
    // SAWP();

    // for(int i = 0; i < 3; i++) {
    //     conveyorMotor.move(-127);
    //     pros::delay(100);
    //     conveyorMotor.move(0);
    //     pros::delay(3000);
    //     descoreMacro();
    //     pros::delay(2000);
    //     chassis.moveToPose(0, 0, 0, 2000, {.forwards = false});
    //     pros::delay(2000);
    // }

    // pros::delay(3000);
    // skills();
    // pros::delay(1000);
    */
	while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * .8;

        if (!forwardOritentiation) {
          leftY = -leftY;
        }
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        if (controller.get_digital_new_press(DIGITAL_X)) {
            forwardOritentiation=!forwardOritentiation;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            pros::Task([](){
                descoreMacro(false);
            });
        }
        if (controller.get_digital(DIGITAL_L2)) {
            conveyorMotor.move(127);
            if (middleGoalBool == LOW) {
                topMotor.move(50);
            } else {
                topMotor.move(127);
            }
        }else if (controller.get_digital(DIGITAL_L1)) {
        	conveyorMotor.move(-127);
        	topMotor.move(-127);  
		}else if (controller.get_digital(DIGITAL_R2)) {
			conveyorMotor.move(127);
            topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            topMotor.brake();
            
        }else if (controller.get_digital(DIGITAL_R1)) {
          	conveyorMotor.move(-127);
          	topMotor.move(-127);  
        }else {
          	conveyorMotor.move(0);
        	topMotor.move(0);
        }
        if (controller.get_digital_new_press(DIGITAL_B)) { // scraper
          scraperBool = !scraperBool;
          scraper.set_value(scraperBool);
        }
        if (controller.get_digital_new_press(DIGITAL_Y)) { // left descore toggle
          leftDescoreBool = !leftDescoreBool;
          leftDescore.set_value(leftDescoreBool);
        }
        if (controller.get_digital_new_press(DIGITAL_UP) && middleGoalBool==LOW) { // upper
        //   middleGoal.set_value(HIGH);
          middleGoalBool=HIGH;
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN) && middleGoalBool==HIGH) {//middle
        //   middleGoal.set_value(LOW);
          middleGoalBool=LOW;
		}
        pros::delay(10);
    }
}

//width 14.6 height 17.5
void autonomous() {
	switch (selectedAuton) {
        case 0: // Middle Control Auto
          	chassis.setPose(-61.3, 16.8, 90);
			leftMiddleControl();
			break;
        case 1: // All Long Goal
            chassis.setPose(-61.3, 16.8, 90);
			leftAllLongGoal();
			break;
        case 2: // Four Ball Rush
            chassis.setPose(-61.3, 16.8, 90);
			leftFourBallRush();
			break;
        case 3: // Right Side
            chassis.setPose(-61.3, -16.8, 90);
			rightSimpleAWP();
			break;
        case 4: // SAWP
            chassis.setPose(-61.3, -16.8, 90);
			SAWP();
			break;
        case 5: // Skills
            chassis.setPose(-47, 15, 0);
			skills();
			break;
        case 6: // User testing or opcontrol
            opcontrol();
            break;
		default:
			break;
	}      
}