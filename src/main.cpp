#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "robodash/api.h"
#include "robodash/views/selector.hpp"
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
bool scraperBool, leftDescoreBool;

bool forwardOritentiation = true;
int descoreCount = 0;

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(5.6, // proportional gain (kP) 6.1
                                              0.135, // integral gain (kI)-.006 over .2
                                              34, // derivative gain (kD) 10
                                              5, // anti windup 1.8
                                              1, // small error range, in inches  1
                                              100, // small error range timeout, in milliseconds 100
                                              3, // large error range, in inches  3
                                              500, // large error range timeout, in milliseconds 500
                                              20 // maximum acceleration (slew)20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.6,   // proportional gain (kP) 1.6
                       0.049,   // integral gain (kI) .0049
                       17,  // derivative gain (kD)//10.7 w/ ki  14
                       25,   // anti windup  25
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
const char* autonNames[] = {"User", "Skills", "Left: Mid SAWP", "Right: Mid SAWP", "SAWP (Long)", "Left: Middle Control", "Left: All Long Goal", "Left: Four Ball Rush", "Right: Middle Control", "Right: All Long Goal", "Right: Four Ball Rush"};

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
            if (selectedAuton > 10) selectedAuton = 10;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            autonSelected = true;
        }

        pros::delay(20);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize(); // initialize brain screen remove for robodash
    chassis.calibrate(); // calibrate sensors
	//pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
    scraperBool = false;
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

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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
    // 0 is user/opcontrol
    // 1 is skills
    // 2 is left middle SAWP
    // 3 is right middle SAWP
    // 4 is SAWP (long)
    // 5 is left middle control
    // 6 is left all long goal
    // 7 is left four ball rush
    // 8 is right middle control
    // 9 is right all long goal
    // 10 is right four ball rush

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    selectedAuton = 9;  //change this everytime
}
void tuningLateralPD() {
  for (int i = 0; i < 4; i++) {
            pros::delay(2000);
            chassis.setPose(0, 0, 0);
    
            chassis.turnToPoint(0, 24, 2000);
            chassis.moveToPoint(0, 24, 2000, {}, false);
            // chassis.turnToPoint(0, 48, 2000);
            // chassis.moveToPoint(0, 48, 2000);
            pros::delay(2000);
            chassis.moveToPose(0, 0, 0, 2000, {.forwards = false});//.earlyExitRange = 0}
        }
        
}
void tuningLateralI() {
        for (int i = 0; i < 4; i++) {
            chassis.turnToPoint(0, 24, 5000);
            chassis.moveToPoint(0, 24, 5000);
            pros::delay(6000);

            chassis.turnToPoint(0, 48, 5000);
            chassis.moveToPoint(0, 48, 5000);
            pros::delay(6000);

            chassis.moveToPose(0, 0, 0, 4000, {.forwards = false});//.earlyExitRange = 0}
        }
        
  
}

void tuningAngularPD() {
  for (int i = 0; i < 8; i++) {
        chassis.turnToHeading(90, 4000);
        pros::delay(2000);
        // chassis.turnToHeading(180, 4000);
        // pros::delay(2000);
        chassis.turnToHeading(0, 4000,  {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  }

  

}
void tuningAngularI() {
  for (int i = 0; i < 5; i++) {
        chassis.turnToHeading(90, 5000);
        pros::delay(5000);
        chassis.turnToHeading(180, 5000);
        pros::delay(5000);
        chassis.turnToHeading(0, 5000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  }
        
}

float goingUnderLongGoalSpeed = 90;
int waitForSwingTime=900;
void leftMiddleControl() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, 46, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // ryan wants to get rid off.
    pros::delay(300);
    scraper.set_value(LOW);

    // Middle Scoring
    chassis.moveToPose(-9, 15, 315, 5000, {.forwards = false, .lead = .3});
    pros::delay(1000);
    topMotor.move(70);
    pros::delay(1500); // make longer if want to score in middle more

    // Going in front of loader.
    int yVal = 52;
    chassis.turnToPoint(-45, yVal, 3000);
    chassis.moveToPoint(-45, yVal, 5000);
    scraper.set_value(HIGH);
    conveyorMotor.move(0);
    topMotor.move(0);
    chassis.turnToHeading(270, 1000);

    // Going into loader 
    chassis.moveToPoint(-54, yVal, 1000, {.minSpeed = 40});
	conveyorMotor.move(127);
    pros::delay(1000);

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-24, yVal, 2000, {.forwards = false});
    conveyorMotor.move(-127);
    pros::delay(700);
    topMotor.move(127);
    pros::delay(1500); // make shorter if needed. 3-4 balls needed.
    
    // Descore
    descoreMacro(true);
}
void leftAllLongGoal() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, 46, 2800, {.maxSpeed = goingUnderLongGoalSpeed});
    pros::delay(300);
    scraper.set_value(LOW);

    // Move back to position to go in front of loader
    chassis.turnToPoint(-14, 40, 2000, {.forwards = false, .minSpeed=10});
    chassis.moveToPoint(-14, 40, 2000, {.forwards = false});
    
    // Going in front of loader
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 2000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    pros::delay(waitForSwingTime + 500);

    // Scoring in long goal
    topMotor.move(-127);
    pros::delay(500);
    topMotor.move(127);
    double yVal = 52;
    chassis.turnToHeading(270, 1000, {}, false);
    pros::delay(1500);
    topMotor.move(0);
    scraper.set_value(HIGH);


    // Going into loader 
    chassis.moveToPoint(-44, yVal, 1000, {.minSpeed = 40});
    pros::delay(1500);

    // Scoring in long goal again
    chassis.moveToPoint(-16, yVal, 2000, {.forwards = false});
    topMotor.move(-127);
    pros::delay(700);
    topMotor.move(127);
    pros::delay(5000);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void leftFourBallRush() {
    // Three balls 
    chassis.moveToPose(-22.22, 22.22, 40, 2000, {.lead = .29, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(300);
    
    // Going in front of long goal
    chassis.turnToPoint(-35, 46, 500, {.forwards = false}); 
    chassis.moveToPoint(-35, 46, 3000, {.forwards = false, .minSpeed = 50});
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-42, 46, 270);   
    scraper.set_value(LOW);

    // Scoring in long goal
    chassis.turnToPoint(-25, 46, 1000, {.forwards = false});
    chassis.moveToPoint(-25, 46, 1000, {.forwards = false});
    topMotor.move(-127);
    pros::delay(500);
    topMotor.move(127);
    pros::delay(2000);
    descoreMacro(true);
}

void rightMiddleControl() {
    // Three balls + 1 under goal (mirrored from leftMiddleControl)
    chassis.moveToPose(-22.22, -22.22, 150, 2000, {.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, -44, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // ryan wants to get rid of maxSpeed.
    pros::delay(300);
    scraper.set_value(LOW);

    // Middle Scoring
    chassis.moveToPoint(-15, -20, 2000, {.forwards = false});
    chassis.turnToPoint(-5, -11, 2000);
    chassis.moveToPoint(-5, -11, 3000, {.forwards = true});
    pros::delay(500);
    topMotor.move(-127);
    conveyorMotor.move(-127);
    pros::delay(1000); // make shorter cuz ryan said so. 2000 too long
    
    // Going in front of loader
    double yVal = -50;
    chassis.turnToPoint(-45, yVal, 3000, {.forwards = false});
    chassis.moveToPoint(-45, yVal, 5000, {.forwards = false});
    conveyorMotor.move(0);
    topMotor.move(0);
    pros::delay(500);
    scraper.set_value(HIGH);
    conveyorMotor.move(0);
    topMotor.move(0);
    chassis.turnToHeading(270, 1000);

    // Going into loader 
    chassis.moveToPoint(-54, yVal, 1000, {.minSpeed = 40});
	conveyorMotor.move(127);
    pros::delay(800);

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-23, yVal, 2000, {.forwards = false});
    conveyorMotor.move(-127);
    pros::delay(700);
    conveyorMotor.move(127);
    topMotor.move(127);
    pros::delay(1500); // make shorter if needed. 3-4 balls needed.
    
    // Descore
    descoreMacro(true);
}
void rightAllLongGoal() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, -22.22, 150, 2000, {.lead = .3, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, -46, 2800, {.maxSpeed = goingUnderLongGoalSpeed});
    pros::delay(300);
    scraper.set_value(LOW);

    // Move back to position to go in front of loader
    chassis.turnToPoint(-14, -40, 2000, {.forwards = false, .minSpeed=10});
    chassis.moveToPoint(-14, -40, 2000, {.forwards = false});
    
    // Going in front of loader
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 2000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    pros::delay(waitForSwingTime + 700);

    // Scoring in long goal
    conveyorMotor.move(-127);
    topMotor.move(-127);
    pros::delay(200);
    conveyorMotor.move(127);
    topMotor.move(127);
    double yVal = -52;
    chassis.turnToHeading(270, 1000, {}, false);
    scraper.set_value(HIGH);
    pros::delay(1500);
    topMotor.move(0);
    

    // Going into loader 
    chassis.moveToPoint(-46, yVal, 1000, {.minSpeed = 40});
    pros::delay(2000);

    // Scoring in long goal again
    chassis.moveToPoint(-16, yVal, 2000, {.forwards = false});
    conveyorMotor.move(-127);
    topMotor.move(-127);
    pros::delay(200);
    conveyorMotor.move(127);
    topMotor.move(127);
    pros::delay(5000);
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void rightFourBallRush() {
    // Three balls
    chassis.moveToPose(-22.22, -22.22, 140, 2000, {.lead = .29, .minSpeed = 50});
    conveyorMotor.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(300);
    
    // Going in front of long goal
    chassis.turnToPoint(-35, -46, 500, {.forwards = false}); 
    chassis.moveToPoint(-35, -46, 3000, {.forwards = false, .minSpeed = 50});
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-42, -46, 270);
    scraper.set_value(LOW);
    
    // Scoring in long goal
    chassis.turnToPoint(-25, -46, 1000, {.forwards = false});
    chassis.moveToPoint(-25, -46, 1000, {.forwards = false});
    topMotor.move(-127);
    conveyorMotor.move(-127);
    pros::delay(200);
    conveyorMotor.move(127);
    topMotor.move(127);
    pros::delay(2000);
    descoreMacro(true);
}

void leftMiddleSAWP() {
    // Goes to loader
    chassis.moveToPoint(-45, 40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
    scraper.set_value(HIGH);
    chassis.moveToPoint(-60, 49, 500, {.minSpeed = 40});
    chassis.turnToPoint(-25, 46, 1000, {.forwards = false});
    chassis.moveToPoint(-54, 46, 500, {.minSpeed = 40});
    conveyorMotor.move(127);
    pros::delay(1000); // possibly to long. Make shorter

    // Score in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, 46, 4000, {.forwards = false});
    pros::delay(600);
    topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1000);
    topMotor.move(0);

    // Get 3 balls
    chassis.moveToPoint(-24, 26, 5000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(200);
    
    // Get the one ball under long goal
    chassis.turnToPoint(-4, 46, 2000);
    chassis.moveToPoint(-4, 46, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // get rid of maxSpeed if ryan wants
    pros::delay(500);
    scraper.set_value(LOW);
    
    // Go to score in middle (pointing backwards for left)
    chassis.moveToPose(-9, 12, 315, 5000, {.forwards = false, .lead = .3});//-7 and 14 is conners
    pros::delay(900);
    topMotor.move(70);
    pros::delay(800); // adjust cause ryan doesn't like it this long
    topMotor.move(0);
    
    // Go back to point of 3 balls and point towards second 3 balls
    chassis.moveToPoint(-24, 26, 3000);
    chassis.turnToPoint(-24, -28, 1000);
    
    // Get the second 3 balls
    chassis.moveToPoint(-24, -28, 2000, {});
    pros::delay(1000);
    scraper.set_value(HIGH);
    
    // Score in middle (for left it should have .forwards = true)
    chassis.turnToPoint(-11, -18, 1000, {.forwards = true});
    conveyorMotor.move(-20);
    chassis.moveToPoint(-11, -18, 5000, {.forwards = true});
    // pros::delay(100);
    conveyorMotor.move(-127);
    topMotor.move(-127);
    pros::delay(1000);
}
void rightMiddleSAWP() {
    // Goes to loader
    chassis.moveToPoint(-45, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
    scraper.set_value(HIGH);
    chassis.moveToPoint(-60, -49, 500, {.minSpeed = 40});
    chassis.turnToPoint(-25, -46, 1000, {.forwards = false});
    chassis.moveToPoint(-54, -46, 500, {.minSpeed = 40});
    conveyorMotor.move(127);
    pros::delay(1000);

    // Score in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, -46, 4000, {.forwards = false});
    pros::delay(600);
    topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1000);
    topMotor.move(0);

    // Get 3 balls
    chassis.moveToPoint(-24, -26, 5000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(200);
    
    // Get the one ball under long goal
    chassis.turnToPoint(-4, -46, 2000);
    chassis.moveToPoint(-4, -46, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // get rid of maxSpeed if ryan wants
    pros::delay(500);
    scraper.set_value(LOW);
    
    // Go to score in middle (pointing forwards for right)
    chassis.moveToPoint(-22.22, -22.22, 2000, {.forwards = false});
    conveyorMotor.move(-20);
    chassis.turnToPoint(-12, -14, 2000);
    chassis.moveToPoint(-12, -14, 3000, {.forwards = true});
    // pros::delay(900); // needed? shorten maybe
    topMotor.move(-127);
    conveyorMotor.move(-127);
    pros::delay(1000); // adjust cause ryan doesn't like it this long
    conveyorMotor.move(0);
    topMotor.move(0);
    
    // Go back to point of 3 balls and point towards second 3 balls
    chassis.moveToPoint(-24, -26, 3000, {.forwards = false});
    chassis.turnToPoint(-24, 28, 1000);
    
    // Get the second 3 balls
    chassis.moveToPoint(-24, 28, 2000, {}, true);
    conveyorMotor.move(127);
    pros::delay(1000);
    scraper.set_value(HIGH);
    
    // Score in middle
    chassis.turnToPoint(-11, 18, 1000, {.forwards = false});
    chassis.moveToPoint(-11, 18, 5000, {.forwards = false});
    pros::delay(100);
    topMotor.move(70);
    pros::delay(2000);
}
void longSAWP() { // place only on right side
    //First loader
    chassis.moveToPoint(-45, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5}); //Position first 
    scraper.set_value(HIGH);
    chassis.moveToPoint(-63, -44, 1000, {.minSpeed = 40});
    conveyorMotor.move(127);
    chassis.turnToHeading(270, 1000);

    //Scoring in first goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-25, -48,2000, {.forwards = false}); //Long goal - X smaller to get closer
    pros::delay(600);
    topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1200);
    
    //First 3 balls
    chassis.moveToPoint(-21, -20, 5000, {.maxSpeed=75, .minSpeed=25, .earlyExitRange=3});
    topMotor.move(0);

    chassis.turnToPoint(-20, 31, 2000, {}, false);
    
    // Second 3 balls
    chassis.moveToPoint(-23, 31, 2000, {});
    pros::delay(700); // adjust
    scraper.set_value(HIGH);
    

    // Middle scoring part
    chassis.turnToPoint(-9,16, 1000, {.forwards = false});
    chassis.moveToPoint(-9, 16, 5000,  {.forwards = false});
    pros::delay(100);
    topMotor.move(70);
    chassis.turnToPoint(-49, 53, 1400);
    pros::delay(500);

    // Second loader approach
    chassis.moveToPoint(-49, 53, 5000);
    topMotor.move(0);

    // Second loader
    chassis.turnToPoint(-54, 53, 1000);
    chassis.moveToPoint(-54, 53, 1000, {.minSpeed = 50});
    chassis.turnToPoint(-25, 53, 2000, {.forwards = false});
    pros::delay(500);

    // Second goal scoring
    chassis.moveToPoint(-25, 53, 5000, {.forwards = false});
    pros::delay(700);
    topMotor.move(127);
    pros::delay(4000);

}

void skills() {
    int run = 0;
    chassis.setPose(-47, 14, 0);

    chassis.turnToPoint(-47, 49, 5000);
    chassis.moveToPoint(-47, 49, 5000); //get in front of loader
    pros::delay(1000);

    for(int i = 0; i < 2; i++){
        if(run != 0){
            chassis.turnToHeading(90, 2000, {}, false); //get balls in loader
            chassis.setPose(-46, 49.5, 270);
        }
        scraper.set_value(HIGH);
        chassis.turnToHeading(270, 2000); //get balls in loader
        conveyorMotor.move(127);
        pros::delay(1000);
        chassis.moveToPoint(-58, 49.5, 1000, {.minSpeed = 50});
        chassis.turnToPoint(-47, 49.5, 1000, {.forwards = false});
        pros::delay(2000);

        chassis.turnToPoint(-47, 49.5, 1000, {.forwards = false});//back up a bit
        chassis.moveToPoint(-47, 49.5, 2000, {.forwards = false});
        scraper.set_value(LOW);
        conveyorMotor.move(0);

        chassis.turnToPoint(-36, 63, 1000);//get in position to go across
        chassis.moveToPoint(-36, 63, 1000, {}, true);
        
        
        chassis.turnToHeading(90, 1000); //go across
        chassis.turnToPoint(0, 63.5, 1000);
        chassis.moveToPoint(0, 63.5, 2000, {.minSpeed = 80});
        chassis.moveToPoint(36, 63.5, 2000, {.minSpeed = 80});

        double yVal = 49.5;

        if(run != 0){
            yVal = 47;
        }
        
        chassis.turnToPoint(47, yVal, 1000); // get in front of loader
        chassis.moveToPoint(47, yVal, 2000);

        chassis.turnToHeading(90, 1000); //score first goal
        chassis.turnToPoint(32, yVal, 1000, {.forwards = false});
        chassis.moveToPoint(32, yVal, 2000, {.forwards = false});
        scraper.set_value(HIGH);
        conveyorMotor.move(-127);     
        pros::delay(500);   
        conveyorMotor.move(127);
        pros::delay(500);
        topMotor.move(127);
        pros::delay(2000);
        topMotor.move(0);

        if(run != 0){
            break;
        }

        // getting balls in loader
        chassis.turnToPoint(65, yVal, 1000);
        chassis.moveToPoint(65, yVal, 2000);
        chassis.turnToPoint(34, yVal, 1000, {.forwards = false, .minSpeed = 50});
        pros::delay(2000);

        //score more
        chassis.moveToPoint(32, yVal, 2000, {.forwards = false});
        chassis.turnToPoint(45, yVal, 1000);
        pros::delay(500);
        topMotor.move(127);
        pros::delay(1000);
        conveyorMotor.move(-127);     
        pros::delay(500);   
        conveyorMotor.move(127);
        pros::delay(2000);
        scraper.set_value(LOW);

        if(run == 0){
            chassis.turnToPoint(45, yVal, 5000);//get in position to go across
            pros::delay(10);
            chassis.moveToPoint(45, yVal, 5000, {.minSpeed=60});
            conveyorMotor.move(0);
            topMotor.move(0);
            chassis.turnToHeading(180, 1000);
            
            chassis.turnToPoint(48, -51, 2000);
            chassis.moveToPoint(48, -51, 5000, {.maxSpeed=100}, false);
            chassis.turnToHeading(180, 1000);
            run++;
        }

    }
    scraper.set_value(LOW);
    chassis.setPose(-32, -47, 270); // reset to right side long goal
    chassis.turnToPoint(-63, -26, 5000);//get readdy to park
    chassis.moveToPoint(-63, -26, 5000);//get readdy to park

    chassis.turnToPoint(-70, -2, 5000);//get readdy to park
    chassis.moveToPoint(-70, -2, 1000, {.minSpeed=127});//parrkkkkkk
    conveyorMotor.move(127);
    pros::delay(300);
    scraper.set_value(HIGH);
    pros::delay(5000);

}

void descoreMacro(bool forward) {
    if (descoreCount == 0 && !forward) {  // Not allowed to run
        return;
    }
    forwardOritentiation = false;
    scraper.set_value(LOW);
    leftDescore.set_value(LOW);
    topMotor.move(0);

    chassis.setPose(0,0,0);
    chassis.moveToPose(14, 9, 3, 750, {.lead = .01, .minSpeed = 100});
    chassis.turnToHeading(3, 1000);
    if(forward){
        chassis.moveToPoint(12, -20, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed=30});
        chassis.turnToHeading(-10, 500);
        pros::delay(100);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    descoreCount = 0;
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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(0,0,0);
    // tuningLateralPD();
    // pros::delay(100);
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
            if (descoreCount == 0) {  // Only run if not already running
                descoreCount = 1;
                pros::Task([](){
                    descoreMacro(false);
                });
            }
        }
         if (controller.get_digital(DIGITAL_L2)) {
            conveyorMotor.move(127);
            topMotor.move(127);
        }else if (controller.get_digital(DIGITAL_L1)) {
        	conveyorMotor.move(127);
            topMotor.move(70);
		}else if (controller.get_digital(DIGITAL_R2)) {
			conveyorMotor.move(127);
            topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);            
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
        
        pros::delay(10);
    }
}
rd::Selector selector("Autonomous Selector", {
    {"User Control", opcontrol},
    {"Skills", skills},
    {"SAWP left", leftMiddleSAWP},
    {"SAWP right", rightMiddleSAWP},
    {"SAWP", longSAWP},
    {"Left Middle", leftMiddleControl},
    {"Left All", leftAllLongGoal},
    {"Left Rush", leftFourBallRush},
    {"Right Middle", rightMiddleControl},
    {"Right All", rightAllLongGoal},
    {"Right Rush", rightFourBallRush}
});
//width 14.6 height 17.5
void autonomous() {
    leftDescore.set_value(HIGH);
    // selector.run_auton();
	switch (selectedAuton) {
        case 0: // User (manual/opcontrol)
            opcontrol();
            break;
        case 1: // Skills
            chassis.setPose(-47, 15, 0);
            skills();
            break;
        case 2: // Left: Middle SAWP
            chassis.setPose(-61.3, 16.8, 90);
            leftMiddleSAWP();
            break;
        case 3: // Right: Middle SAWP
            chassis.setPose(-61.3, -16.8, 90);
            rightMiddleSAWP();
            break;
        case 4: // SAWP (Long)
            chassis.setPose(-61.3, -16.8, 90);
            longSAWP();
            break;
        case 5: // Left: Middle Control
            chassis.setPose(-61.3, 16.8, 90);
            leftMiddleControl();
            break;
        case 6: // Left: All Long Goal
            chassis.setPose(-61.3, 16.8, 90);
            leftAllLongGoal();
            break;
        case 7: // Left: Four Ball Rush
            chassis.setPose(-61.3, 16.8, 90);
            leftFourBallRush();
            break;
        case 8: // Right: Middle Control
            chassis.setPose(-61.3, -16.8, 90);
            rightMiddleControl();
            break;
        case 9: // Right: All Long Goal
            chassis.setPose(-61.3, -16.8, 90);
            rightAllLongGoal();
            break;
        case 10: // Right: Four Ball Rush
            chassis.setPose(-61.3, -16.8, 90);
            rightFourBallRush();
            break;
        default:
            break;
    }
    // pros::delay(4000); // get rid off during comp.
    topMotor.move(0);
    // conveyorMotor.move(0);

}