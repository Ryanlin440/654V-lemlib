#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

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
lemlib::ControllerSettings lateral_controller(4.9, // proportional gain (kP)
                                              .1, // integral gain (kI)-.006 over .41
                                              24, // derivative gain (kD)
                                              4.25, // anti windup 1.8
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              30 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings
    angular_controller(1.9,   // proportional gain (kP)
                       .0015,   // integral gain (kI) .03 too little
                       12,  // derivative gain (kD)//10.7 w/ ki
                       3.25184,   // anti windup  3
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
const char* autonNames[] = {"Left", "Right", "SAWP", "Skills", "User"};
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
            if (selectedAuton > 4) selectedAuton = 4;
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
    middleGoalBool = true;
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
    // autonSelector();//remove this aswell
    // autonomous();//⚠️ Remove this before competitions.
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
  autonSelector();

}


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
void testing() {
  	chassis.setPose(0, 0, 0);
        // pros::delay(3000);
        chassis.moveToPoint(0, 24, 2000);
        // chassis.moveToPose(0, 24,0,4000);
        pros::delay(6000);
        chassis.moveToPoint(0, 48, 2000);
        pros::delay(6000);

        // // chassis.turnToHeading(0, 5000);
        // // pros::delay(2000);
        chassis.moveToPose(0, 0, 0, 4000, {.forwards = false});//.earlyExitRange = 0}

        // chassis.turnToHeading(90, 5000);
        // chassis.turnToHeading(180, 5000);
        // chassis.turnToHeading(0, 5000);

    
    
}
void goAroundObstacle() {
    chassis.setPose(0, 0, 90);

topMotor.move(60);

chassis.moveToPose(
    48, -24, 90,
    2000,
    {.minSpeed = 72}   // NO earlyExitRange
);

chassis.moveToPose(
    64, 3, 0,
    2000,
    {.minSpeed = 60}
);

topMotor.move(-60);
pros::delay(2000);


}

void leftSimpleAWP() {
	chassis.moveToPose(-22.22, 22.22, 26.2,2000,{.lead = .3, .minSpeed = 50}); //{.horizontalDrift = 8, .lead = 0.3}
	conveyorMotor.move(100);
    chassis.moveToPose(-5.5, 51, 80, 5000, {.lead = .05, .maxSpeed=50});//.15

    // pros::delay(3000);
    chassis.moveToPose(-14, 14, 315, 5000, {.forwards = false, .lead = .3},
                       false);
    pros::delay(3000);
    // middleGoal.set_value(LOW);
    // pros::delay(2000);
    // middleGoal.set_value(HIGH);
    // conveyorMotor.move(0);

    // chassis.turnToHeading(315, 3000);
    // chassis.moveToPoint(-47, 47, 5000);

    // chassis.turnToHeading(270, 1000);
    // chassis.moveToPoint(-56, 47, 2000);
	// conveyorMotor.move(127);
    // pros::delay(2000);

    // chassis.turnToHeading(270, 1000);
    // chassis.moveToPoint(-33, 47, 2000, {.forwards = false, .minSpeed = 40});
    // pros::delay(1000);
    // topMotor.move(127);
	// pros::delay(1000);
	// conveyorMotor.move(0);
    // topMotor.move(0);
    // chassis.moveToPoint(-37, 57, 2000, {});
    // chassis.turnToPoint(-17, 57, 1000, {.forwards = false});
    // chassis.moveToPoint(-17, 57, 2000, {.forwards = false});
    // chassis.turnToHeading(270, 500);
    // // chassis.moveToPoint(-15, 57, 5000, {.forwards = false});
    chassis.turnToPoint(-61.3, 16.8, 2000);
    chassis.moveToPoint(-61.3, 16.8, 8000, {});

    chassis.turnToHeading(90, 1000);
    conveyorMotor.move(-127);
    pros::delay(2000);
}
void rightSimpleAWP() {}//mirror y values
void SAWP() {//place only on right side
	chassis.moveToPoint(-54.998, -47.634, 5000);
	chassis.moveToPoint(-27.843, -47.848, 5000, {.forwards = false});
	chassis.moveToPoint(-23.139, -22.404, 5000);
	chassis.moveToPoint(-22.284, 22.07, 5000);
	chassis.moveToPoint(-11.379, 11.593, 5000, {.forwards = false});
	chassis.moveToPoint(-55.426, 45.162, 5000);
	chassis.moveToPoint(-32.761, 46.018, 5000, {.forwards = false});

}
void skills() {
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
  // testing();
    chassis.setPose(-61.3, 16.8, 90);
  leftSimpleAWP();
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
        if (controller.get_digital(DIGITAL_L2)) {
        	conveyorMotor.move(127);
        	topMotor.move(127);

        }else if (controller.get_digital(DIGITAL_L1)) {
        	conveyorMotor.move(-127);
        	topMotor.move(-127);  
		}else if (controller.get_digital(DIGITAL_R2)) {
			conveyorMotor.move(127);
                        // topMotor.move(-15);
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
          middleGoal.set_value(HIGH);
          middleGoalBool=HIGH;
        }
        if (controller.get_digital_new_press(DIGITAL_DOWN) && middleGoalBool==HIGH) {//middle
          middleGoal.set_value(LOW);
          middleGoalBool=LOW;
		}
        pros::delay(10);
    }
}

void autonomous() {//width 14.6 height 17.5
	switch (selectedAuton) {
        case 0: // left
            // testing();
          	chassis.setPose(-61.3, 16.8, 90);
			leftSimpleAWP();
			break;
        case 1://right
            chassis.setPose(-61.3, -16.8, 90);//this is for left right now please change in the future
			rightSimpleAWP(); //put this back when done testing
			break;
        case 2://SAWP 
            chassis.setPose(-61.3, -16.8, 90);
			SAWP();
			break;
        case 3://skills
            
			skills();//put this back when done testing
			break;
        case 4: // testing or opcontrol
            //  goAroundObstacle() ;
            //  testing();
            opcontrol();
            break;
		default:
			break;
	}      
}