#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "robodash/api.h"
#include "robodash/views/selector.hpp"
#include "pros/rtos.hpp"
void descoreMacro(bool forward);
void scoreTop();
void scoreMiddle();
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
pros::MotorGroup right_motor_group({19, 17, 20}, pros::MotorGears::blue);

lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              10.6,                    // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              800, // drivetrain rpm is 800
                              8    // horizontal drift is 2 (for now),
                              
);

// create an imu on port 8
pros::Imu imu(16);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(14);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,-.5);
pros::Rotation horizontal_encoder(12);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);

pros::Motor intake (9, pros::MotorGears::blue);
pros::Motor armMotor(-13, pros::MotorGears::red);

pros::adi::DigitalOut scraper('h',LOW);
pros::adi::DigitalOut middleGoal('g', LOW);
pros::adi::DigitalOut wing('e',LOW);
pros::adi::DigitalOut hood('f',LOW);

bool scraperBool, wingBool, middleGoalBool, hoodBool;

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
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP) 6.1
                                              0.1, // integral gain (kI)-.006 over .0.135
                                              40, // derivative gain (kD) 34
                                              5, // anti windup 5
                                              1, // small error range, in inches  1
                                              100, // small error range timeout, in milliseconds 100
                                              3, // large error range, in inches  3
                                              50, // large error range timeout, in milliseconds 500
                                              0 // maximum acceleration (slew)20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP) 1.6
                       .05,   // integral gain (kI) .0049
                       18,  // derivative gain (kD)//10.7 w/ ki  17
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

    pros::lcd::register_btn1_cb(on_center_button);
    scraperBool = false;
    wingBool = false;
    middleGoalBool = false;
    hoodBool = false;
    pros::Task screenTask([&]() {
      while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(3, "arm Deg: %f", armMotor.get_position());
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
      }
    });

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // opcontrol();
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
    
    selectedAuton = 7;  //change this everytime
    autonSelector();
}
void tuningLateralPD() {
  for (int i = 0; i < 4; i++) {
            pros::delay(100);
            chassis.setPose(0, 0, 0);
    
            chassis.turnToPoint(0, 48, 2000);
            chassis.moveToPoint(0, 48, 2000, {}, false);
            // chassis.turnToPoint(0, 48, 2000);
            // chassis.moveToPoint(0, 48, 2000);
            pros::delay(1500);
            chassis.moveToPose(0, 0, 0, 2000,
                               {.forwards = false}); //.earlyExitRange = 0}
            pros::delay(3000);
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

float goingUnderLongGoalSpeed = 60;
int waitForSwingTime=900;
void leftMiddleControl() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-2, 48, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // ryan wants to get rid off.
    pros::delay(300);
    scraper.set_value(LOW);
    pros::delay(500);

    // Middle Scoring
    chassis.moveToPose(-6, 15, 315, 5000, {.forwards = false, .lead = .3}, false);
    scoreMiddle();

    // Going in front of loader.
    int yVal = 54;
    chassis.turnToPoint(-45, yVal, 3000);
    chassis.moveToPoint(-45, yVal, 5000);
    scraper.set_value(HIGH);
    intake.move(0);
    chassis.turnToHeading(270, 1000);

    // Going into loader 
    chassis.moveToPoint(-54, yVal, 3000, {.minSpeed = 90});
    intake.move(127);
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    pros::delay(1000);

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-22, yVal, 2000, {.forwards = false});
    pros::delay(700);
    scoreTop();
    pros::delay(500);
    
    // Descore
    descoreMacro(true);
}
void leftAllLongGoal() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, 22.22, 30, 2000,{.lead = .3, .minSpeed = 50});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, 46, 2800, {.maxSpeed = goingUnderLongGoalSpeed});
    pros::delay(300);
    scraper.set_value(LOW);

    // Move back to position to go in front of loader
    chassis.turnToPoint(-14, 41, 2000, {.forwards = false, .minSpeed=10});
    chassis.moveToPoint(-14, 41, 2000, {.forwards = false});

    // Going in front of loader
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    chassis.swingToHeading(270, lemlib::DriveSide::LEFT, 2000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE});
    pros::delay(waitForSwingTime);

    // Scoring in long goal
    ////topMotor.move(127);
    scoreTop();
    int yVal = 52;
    chassis.turnToHeading(270, 1000, {}, false);
    // pros::delay(300);
    //topMotor.move(0);
    scraper.set_value(HIGH);


    // // Going into loader 
    chassis.moveToPoint(-44, yVal, 1000, {.minSpeed = 40});
    pros::delay(1500);

    // Scoring in long goal again
    chassis.moveToPoint(-16, yVal, 2000, {.forwards = false});
    pros::delay(700);
    scoreTop();
    // pros::delay(300);
    intake.move(0);
    //topMotor.move(0);

    // Bumping
    // chassis.moveToPoint(-35, yVal, 2000);
    // chassis.moveToPoint(-15, yVal, 2000, {.forwards = false, .minSpeed = 90}, false); // Make faster if needed
    // pros::delay(500);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void leftFourBallRush() {
  // Three balls
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    chassis.moveToPose(-23.22, 25.22, 45, 2000, {.lead = .29, .minSpeed = 50});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(500);
    
    // Going in front of long goal
    chassis.turnToPoint(-35, 48, 500, {.forwards = false}); 
    chassis.moveToPoint(-35, 48, 3000, {.forwards = false, .minSpeed = 50});
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-42, 48, 270);   
    scraper.set_value(LOW);

    // Scoring in long goal
    chassis.turnToPoint(-28, 48, 1000, {.forwards = false});
    chassis.moveToPoint(-28, 48, 1000, {.forwards = false});
    pros::delay(800);
    scoreTop();
    pros::delay(500);
    scoreTop();
    pros::delay(500);
    descoreMacro(true);
}

void rightMiddleControl() {
    // Three balls + 1 under goal (mirrored from leftMiddleControl)
    chassis.moveToPose(-22.22, -22.22, 150, 2000, {.lead = .3});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, -48, 2800, {.maxSpeed = goingUnderLongGoalSpeed}); // ryan wants to get rid of maxSpeed.
    pros::delay(300);
    scraper.set_value(LOW);
    pros::delay(500);
    middleGoal.set_value(HIGH);
    middleGoalBool=true;

    // Middle Scoring
    chassis.moveToPoint(-15, -22.22, 2000, {.forwards = false});
    chassis.turnToPoint(-3, -16, 2000);
    chassis.moveToPoint(-3, -16, 2000, {.forwards = true});
    pros::delay(300);
    intake.move(-127);
    pros::delay(1500); // make shorter cuz ryan said so. 2000 too long
    
    // Going in front of loader
    int yVal = -54;
    chassis.turnToPoint(-45, yVal, 3000, {.forwards = false});
    chassis.moveToPoint(-45, yVal, 3000, {.forwards = false});
    scraper.set_value(HIGH);
    intake.move(0);
    chassis.turnToHeading(270, 1000);

    // Going into loader 
    chassis.moveToPoint(-54, yVal, 3000, {.minSpeed = 90});
    intake.move(127);
    pros::delay(1000);

    // Scoring in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-22, yVal, 2000, {.forwards = false});
    pros::delay(700);
    scoreTop();
    pros::delay(500);
    
    // Descore
    descoreMacro(true);
}
void rightAllLongGoal() {
    // Three balls + 1 under goal
    chassis.moveToPose(-22.22, -22.22, 150, 2000, {.lead = .3, .minSpeed = 50});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    chassis.moveToPoint(-4, -46, 2800, {.maxSpeed = goingUnderLongGoalSpeed});
    pros::delay(500);
    scraper.set_value(LOW);

    // Move back to position to go in front of loader
    chassis.turnToPoint(-14, -40.5, 2000, {.forwards = false, .minSpeed=10});
    chassis.moveToPoint(-14, -40.5, 2000, {.forwards = false});

    // Going in front of loader
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    chassis.swingToHeading(270, lemlib::DriveSide::RIGHT, 2000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    pros::delay(waitForSwingTime);

    // Scoring in long goal
    // topMotor.move(127);
    scoreTop();
    int yVal = -52;
    chassis.turnToHeading(270, 1000, {}, false);
    // pros::delay(1500);
    //topMotor.move(0);
    scraper.set_value(HIGH);


    // // Going into loader
    chassis.moveToPoint(-45, yVal, 1000, {.minSpeed = 40});
    intake.move(127);
    pros::delay(1500);

    // Scoring in long goal again
    chassis.moveToPoint(-16, yVal, 2000, {.forwards = false});
    pros::delay(700);
    // topMotor.move(127);
    scoreTop();
    // pros::delay(1000);
    // intake.move(0);
    //topMotor.move(0);

    // Bumping
    // chassis.moveToPoint(-35, yVal, 2000);
    // chassis.moveToPoint(-15, yVal, 2000, {.forwards = false, .minSpeed = 90}, false); // Make faster if needed
    // pros::delay(500);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
void rightFourBallRush() {
    // Three balls
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    chassis.moveToPose(-22.22, -22.22, 140, 2000, {.lead = .29, .minSpeed = 50});
    intake.move(127);
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(300);
    
    // Going in front of long goal
    chassis.turnToPoint(-35, -45, 500, {.forwards = false}); 
    chassis.moveToPoint(-35, -45, 3000, {.forwards = false, .minSpeed = 50});
    chassis.turnToHeading(270, 1000, {}, false);
    chassis.setPose(-42, -45, 270);
    scraper.set_value(LOW);
    
    // Scoring in long goal
    chassis.turnToPoint(-25, -45, 1000, {.forwards = false});
    chassis.moveToPoint(-25, -45, 1000, {.forwards = false});
    pros::delay(800);
    scoreTop();
    pros::delay(500);
    descoreMacro(true);
}

void leftMiddleSAWP() {
    // Goes to loader
    chassis.moveToPoint(-45, 40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
    scraper.set_value(HIGH);
    chassis.moveToPoint(-60, 49, 500, {.minSpeed = 40});
    chassis.turnToPoint(-25, 46, 1000, {.forwards = false});
    chassis.moveToPoint(-54, 46, 500, {.minSpeed = 40});
    intake.move(127);
    pros::delay(1000); // possibly to long. Make shorter

    // Score in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, 46, 4000, {.forwards = false});
    pros::delay(600);
    //topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1000);
    //topMotor.move(0);

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
    //topMotor.move(70);
    pros::delay(800); // adjust cause ryan doesn't like it this long
    //topMotor.move(0);
    
    // Go back to point of 3 balls and point towards second 3 balls
    chassis.moveToPoint(-24, 26, 3000);
    chassis.turnToPoint(-24, -28, 1000);
    
    // Get the second 3 balls
    chassis.moveToPoint(-24, -28, 2000, {});
    pros::delay(1000);
    scraper.set_value(HIGH);
    
    // Score in middle (for left it should have .forwards = true)
    chassis.turnToPoint(-11, -18, 1000, {.forwards = true});
    intake.move(-20);
    chassis.moveToPoint(-11, -18, 5000, {.forwards = true});
    // pros::delay(100);
    intake.move(-127);
    //topMotor.move(-127);
    pros::delay(1000);
}
void rightMiddleSAWP() {
    // Goes to loader
    chassis.moveToPoint(-45, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5});
    scraper.set_value(HIGH);
    chassis.moveToPoint(-60, -49, 500, {.minSpeed = 40});
    chassis.turnToPoint(-25, -46, 1000, {.forwards = false});
    chassis.moveToPoint(-54, -46, 500, {.minSpeed = 40});
    intake.move(127);
    pros::delay(1000);

    // Score in long goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-27, -46, 4000, {.forwards = false});
    pros::delay(600);
    //topMotor.move(127);
    scraper.set_value(LOW);
    pros::delay(1000);
    //topMotor.move(0);

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
    intake.move(-20);
    chassis.turnToPoint(-12, -14, 2000);
    chassis.moveToPoint(-12, -14, 5000, {.forwards = true});
    // pros::delay(900); // needed? shorten maybe
    //topMotor.move(-127);
    intake.move(-127);
    pros::delay(1000); // adjust cause ryan doesn't like it this long
    intake.move(0);
    //topMotor.move(0);
    
    // Go back to point of 3 balls and point towards second 3 balls
    chassis.moveToPoint(-24, -26, 3000, {.forwards = false});
    chassis.turnToPoint(-24, 28, 1000);
    
    // Get the second 3 balls
    chassis.moveToPoint(-24, 28, 2000, {}, true);
    intake.move(127);
    pros::delay(1000);
    scraper.set_value(HIGH);
    
    // Score in middle
    chassis.turnToPoint(-11, 18, 1000, {.forwards = false});
    chassis.moveToPoint(-11, 18, 5000, {.forwards = false});
    pros::delay(100);
    //topMotor.move(70);
    pros::delay(2000);
}
void longSAWP() { // place only on right side
                  // First loader
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    chassis.moveToPoint(-45, -40, 2000, {.minSpeed = 60, .earlyExitRange = 5}); //Position first 
    scraper.set_value(HIGH);
    chassis.moveToPoint(-68, -44, 500, {.minSpeed = 40});
    // chassis.turnToPoint(-25, -46, 1000, {.forwards = false});
    // chassis.moveToPoint(-54, -46, 500, {.minSpeed = 40});
    intake.move(127);
    chassis.turnToHeading(270, 1000);
    pros::delay(1000);

    //Scoring in first goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-25, -44,4000, {.forwards = false}); //Long goal - X smaller to get closer
    pros::delay(600);
    // topMotor.move(127);
    scoreTop();
    scraper.set_value(LOW);
    // pros::delay(1000);
    //topMotor.move(0);

    // First 3 balls
    intake.move(127);
    chassis.moveToPoint(-27, -21, 5000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800); // possibly need to adjust
    scraper.set_value(HIGH);
    pros::delay(200);
    chassis.turnToPoint(-23, 31, 2000, {}, false);
    // pros::delay(1000000);
    scraper.set_value(LOW);

    // Second 3 balls
    chassis.moveToPoint(-23, 31, 2000, {});
    pros::delay(1000); // adjust
    scraper.set_value(HIGH);
    middleGoal.set_value(LOW);
    middleGoalBool=false;
    // intake.move(0);
    

    //// Middle scoring part
    chassis.turnToPoint(-11,18, 1000, {.forwards = false});
    chassis.moveToPoint(-11, 18, 5000,  {.forwards = false});
    pros::delay(100);
    scoreMiddle();
    chassis.turnToPoint(-49, 57, 1400);
    // pros::delay(1500);

    // Second loader approach
    chassis.moveToPoint(-49, 57, 5000);
    // topMotor.move(0);
    intake.move(127);

    // Second loader
    chassis.turnToPoint(-60, 57, 1000);
    chassis.moveToPoint(-60, 57, 3000);
    middleGoal.set_value(HIGH);
    middleGoalBool=true;
    pros::delay(1000);

    // Second goal scoring
    chassis.turnToPoint(-32, 57, 2000, {.forwards = false});
    chassis.moveToPoint(-32, 57, 5000, {.forwards = false});
    scoreTop();
    pros::delay(700);
    //topMotor.move(127);

}

void skills() {

    int run = 0;
    chassis.setPose(-47, 14, 0);

    chassis.turnToPoint(-47, 49, 5000);
    chassis.moveToPoint(-47, 49, 5000); //get in front of loader
    scraper.set_value(HIGH);
    pros::delay(1500);

    for(int i = 0; i < 1; i++){
        if(run != 0){
            chassis.setPose(-47, 49, 0);
        }
        chassis.turnToHeading(270, 2000); //get balls in loader
        intake.move(127);
        chassis.moveToPoint(-59, 49, 1000, {.minSpeed = 50});
        chassis.turnToPoint(-47, 49, 1000, {.forwards = false});
        pros::delay(2000);

        chassis.turnToPoint(-47, 49, 5000, {.forwards = false});//back up a bit
        chassis.moveToPoint(-47, 49, 5000, {.forwards = false});
        // scraper.set_value(LOW);
        intake.move(0);

        chassis.turnToPoint(-36, 63, 1000);//get in position to go across
        chassis.moveToPoint(-36, 63, 1000, {}, true);
        
        
        chassis.turnToHeading(90, 1000); //go across
        chassis.turnToPoint(0, 63.5, 2000);
        chassis.moveToPoint(0, 63.5, 5000, {.minSpeed = 50});
        chassis.turnToPoint(36, 63.5, 2000, {});
        chassis.moveToPoint(36, 63.5, 5000, {.minSpeed = 50});

        int yVal = 49;
        chassis.turnToPoint(47, yVal, 2000); // get in front of loader
        chassis.moveToPoint(47, yVal, 5000);

        chassis.turnToHeading(90, 1000); //score first goal
        chassis.turnToPoint(34, yVal, 2000, {.forwards = false});
        chassis.moveToPoint(34, yVal, 5000, {.forwards = false});
        // scraper.set_value(HIGH);
        intake.move(127);
        pros::delay(500);
        //topMotor.move(127);
        pros::delay(2000);
        //topMotor.move(0);

        // getting balls in loader
        chassis.turnToPoint(65, yVal, 5000);
        chassis.moveToPoint(65, yVal, 5000);
        chassis.turnToPoint(34, yVal, 10000, {.forwards = false, .minSpeed = 50});
        pros::delay(2000);

        //score more
        chassis.moveToPoint(34, yVal, 5000, {.forwards = false});
        chassis.turnToPoint(45, yVal, 2000);
        pros::delay(500);
        //topMotor.move(127);
        pros::delay(2000);

        if(run == 1){
            chassis.turnToPoint(45, yVal, 5000);//get in position to go across
            pros::delay(10);
            chassis.moveToPoint(45, yVal, 5000, {.minSpeed=60});
            intake.move(0);
            //topMotor.move(0);
            chassis.turnToHeading(180, 1000);
            run++;

            chassis.turnToPoint(45, -yVal, 2000);
            chassis.moveToPoint(45, -yVal, 5000, {.maxSpeed=50});
            pros::delay(100);
        }

    }
    

    chassis.setPose(-32, -47, 0); // reset to right side long goal
    chassis.turnToPoint(-60, -23, 5000);//get readdy to park
    chassis.moveToPoint(-60, -23, 5000);//get readdy to park
    intake.move(127);

    chassis.turnToPoint(-65, 0, 5000);//get readdy to park
    chassis.moveToPoint(-65, 0, 5000);//parrkkkkkk
    pros::delay(500);
    scraper.set_value(HIGH);

}

void descoreMacro(bool forward) {
    forwardOritentiation = false;
    scraper.set_value(LOW);
    wing.set_value(LOW);
    intake.move(0);

    chassis.setPose(0,0,0);
    chassis.moveToPose(13, 6, 3, 2000, {.lead = .01, .minSpeed = 100});
    chassis.turnToHeading(5, 1000);
    if(forward){
        chassis.moveToPoint(11, -20, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed=30});
        chassis.turnToHeading(-10, 500);
        pros::delay(100);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(0,0,0);
    // tuningLateralPD();
    // pros::delay(100);
	while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (!forwardOritentiation) {
          leftY = -leftY;
        }
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if (controller.get_digital_new_press(DIGITAL_X)) {
            forwardOritentiation=!forwardOritentiation;
        }
        if (controller.get_digital(DIGITAL_L2)) {
            scoreTop();
            hood.set_value(HIGH);
            hoodBool=true;
        }else if (controller.get_digital(DIGITAL_L1)) {
            scoreMiddle();
            hood.set_value(HIGH);
            hoodBool=true;
		}else if (controller.get_digital(DIGITAL_R2)) {
			intake.move(127);
            hood.set_value(LOW);
            hoodBool=false;
        }else if (controller.get_digital(DIGITAL_R1)) {
          	intake.move(-127);
        }else {
          	intake.move(0);
        	
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            hoodBool=!hoodBool;
            hood.set_value(hoodBool);
        }
        if (controller.get_digital_new_press(DIGITAL_B)) { // scraper
          scraperBool = !scraperBool;
          scraper.set_value(scraperBool);
        }
        if (controller.get_digital_new_press(DIGITAL_Y)) { // left descore toggle
          wingBool = !wingBool;
          wing.set_value(wingBool);
          scraperBool = false;
          scraper.set_value(scraperBool);
        }
        if (controller.get_digital_new_press(DIGITAL_UP) && middleGoalBool == false) {
          middleGoalBool = true;
          middleGoal.set_value(HIGH);
        } else if (controller.get_digital_new_press(DIGITAL_DOWN) && middleGoalBool == true) {
          middleGoalBool = false;
          middleGoal.set_value(LOW);
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
    //topMotor.move(0);
    // intake.move(0);
}

void scoreTop() {
    int speedForTop = 127;
    if(selectedAuton == 1){
        speedForTop = 100;
    }
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
    armMotor.move_absolute(620, 127);
    pros::delay(300);
    armMotor.move_absolute(15, 40);
}
void scoreMiddle() {
    int speedForMiddle = 60;
    if(selectedAuton == 1){
        speedForMiddle = 40;
    }
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
    armMotor.move_absolute(610, 60);
    pros::delay(500);
    armMotor.move_absolute(15, 40);
}