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
void scoreTop();
void scoreMiddle();
void park();
/*
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
const char* autonNames[] = {"User", "Skills", "SAWP (Long)", "Left: Mid Control", "Right: Mid Control"};

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
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize(); // initialize brain screen. Remove for robodash
    chassis.calibrate(); // calibrate sensors

    // pros::lcd::register_btn1_cb(on_center_button);
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
        
    // selectedAuton = 0;  //change this everytime
    // autonSelector();
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

void leftMiddleControl() {
    chassis.setPose(-47, 14, 0);
    middleGoal.set_value(HIGH);

    //get in front of loader
    chassis.turnToPoint(-47, 47, 2000);
    chassis.moveToPoint(-47, 47, 2000, {.minSpeed=70}); 
    scraper.set_value(HIGH);
    pros::delay(500);

    //get balls in loader
    chassis.turnToHeading(270, 2000); 
    intake.move(127);
    chassis.moveToPoint(-59, 47, 1000, {.minSpeed = 70}, false);
    chassis.turnToPoint(-30, 47, 1000, {.forwards = false}, false);
    pros::delay(500);

    //Go to score
    chassis.moveToPoint(-30, 47, 2000, {.forwards = false, .minSpeed=90}, false);
    scraper.set_value(LOW);
    scoreTop();

    // Get 3 balls
    chassis.moveToPoint(-24, 26, 2000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800);
    scraper.set_value(HIGH);
    middleGoal.set_value(LOW);
    pros::delay(500);

    // Go score in middle
    chassis.turnToHeading(315, 1000);
    chassis.moveToPoint(-13, 13, 2000, {.forwards = false}, false);
    pros::delay(500);
    scraper.set_value(LOW);
    scoreMiddle();

    // Go wing
    intake.move(0);
    middleGoal.set_value(HIGH);
    chassis.moveToPoint(-29, 37, 2000, {.minSpeed=70});
    chassis.turnToHeading(70, 1000);
    chassis.moveToPoint(-8, 40, 2000, {.minSpeed=50});
    chassis.turnToHeading(110, 1000);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void rightMiddleControl() {
    chassis.setPose(-47, -14, 180);
    middleGoal.set_value(HIGH);

    //get in front of loader
    chassis.turnToPoint(-47, -47, 2000);
    chassis.moveToPoint(-47, -47, 2000, {.minSpeed=70}); 
    scraper.set_value(HIGH);
    pros::delay(500);

    //get balls in loader
    chassis.turnToHeading(270, 2000); 
    intake.move(127);
    chassis.moveToPoint(-59, -47, 1000, {.minSpeed = 70}, false);
    chassis.turnToPoint(-30, -47, 1000, {.forwards = false}, false);
    pros::delay(500);

    //Go to score
    chassis.moveToPoint(-30, -47, 2000, {.forwards = false, .minSpeed=90}, false);
    scraper.set_value(LOW);
    scoreTop();

    // Get 3 balls
    chassis.moveToPoint(-24, -26, 2000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800);
    scraper.set_value(HIGH);
    pros::delay(500);

    // Go score in middle
    chassis.moveToPoint(-11, -11, 2000);
    scraper.set_value(LOW);
    pros::delay(500);
    intake.move(-127);
    pros::delay(1500);

    // Go wing
    intake.move(0);
    chassis.moveToPoint(-29, -37, 2000, {.minSpeed=70});
    chassis.turnToHeading(110, 1000);
    chassis.moveToPoint(-8, -40, 2000, {.minSpeed=50});
    chassis.turnToHeading(70, 1000);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void longSAWP() { // place only on right side
    chassis.setPose(-47, -14, 180);

    //get in front of loader
    chassis.turnToPoint(-47, -47, 2000);
    chassis.moveToPoint(-47, -47, 2000, {.minSpeed=70}); 
    scraper.set_value(HIGH);

    //get balls in loader
    chassis.turnToHeading(270, 2000); 
    intake.move(127);
    chassis.moveToPoint(-59, -47, 1000, {.minSpeed = 70}, false);
    chassis.turnToPoint(-47, -47, 1000, {.forwards = false}, false);
    pros::delay(500);

    //Scoring in first goal
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-32, -47,2000, {.forwards = false}, false);
    scoreTop();
    scraper.set_value(LOW);

    // First 3 balls
    intake.move(127);
    chassis.moveToPoint(-24, -26, 2000, {.maxSpeed=75, .minSpeed=30, .earlyExitRange=1});
    pros::delay(800); // possibly need to adjust
    scraper.set_value(HIGH);
    pros::delay(500);
    chassis.turnToPoint(-23, 26, 2000, {}, false);
    scraper.set_value(LOW);

    // Second 3 balls
    chassis.moveToPoint(-23, 26, 2000, {});
    pros::delay(1000); // adjust
    scraper.set_value(HIGH);
    middleGoal.set_value(LOW);
    

    //// Middle scoring part
    chassis.turnToPoint(-13,13, 1000, {.forwards = false});
    chassis.moveToPoint(-13, 13, 2000, {.forwards = false}, false);
    pros::delay(500);
    scoreMiddle();
    chassis.turnToPoint(-49, 47, 1400);

    // Second loader approach
    chassis.moveToPoint(-49, 47, 5000);

    // Second loader
    chassis.turnToPoint(-59, 47, 1000);
    chassis.moveToPoint(-59, 47, 3000, {.minSpeed = 80});
    middleGoal.set_value(HIGH);
    pros::delay(1000);

    // Second goal scoring
    chassis.turnToPoint(-32, 47, 2000, {.forwards = false});
    chassis.moveToPoint(-32, 47, 5000, {.forwards = false});
    scoreTop();
    pros::delay(700);

}

void skills() {
    int run = 0;
    selectedAuton = 1;
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

void moveForward(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 5, 2000);
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
    scraperBool = false;
    wingBool = false;
    middleGoalBool = false;
    hoodBool = false;
    selectedAuton = 0;

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
        if (controller.get_digital(DIGITAL_L2)) { // top score
            scoreTop();
            
        }else if (controller.get_digital(DIGITAL_L1)) { // middle score
            scoreMiddle();
            
		}else if (controller.get_digital(DIGITAL_R2)) { // intake
            hood.set_value(LOW);
            hoodBool=false;
			intake.move(127);
            
        }else if (controller.get_digital(DIGITAL_R1)) { // outake
          	intake.move(-127);
        }else {
          	intake.move(0);
        	
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){ // hood
            hoodBool=!hoodBool;
            hood.set_value(hoodBool);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            park();
        }
        if (controller.get_digital_new_press(DIGITAL_B)) { // scraper
          scraperBool = !scraperBool;
          scraper.set_value(scraperBool);
        }
        if (controller.get_digital(DIGITAL_Y)) { // left while held down
          wingBool = true;
          wing.set_value(wingBool);
          scraperBool = false;
          scraper.set_value(scraperBool);
        }else{ // down when not pressed
            wingBool = false;
            wing.set_value(wingBool);
        }
        if (controller.get_digital_new_press(DIGITAL_UP) && middleGoalBool == false) { // Going up
          middleGoalBool = true;
          middleGoal.set_value(HIGH);
        } else if (controller.get_digital_new_press(DIGITAL_DOWN) && middleGoalBool == true) { // Going down
          middleGoalBool = false;
          middleGoal.set_value(LOW);
          wingBool = false;
          wing.set_value(wingBool);
        }
        
        pros::delay(10);
    }
}
rd::Selector selector("Autonomous Selector", {
    {"User Control", opcontrol},
    {"Skills", skills},
    {"SAWP", longSAWP},
    {"Left Middle", leftMiddleControl},
    {"Right Middle", rightMiddleControl},
    {"Move Forward", moveForward}

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
        case 2: // SAWP (Long)
            chassis.setPose(-46, 0, 0);
            longSAWP();
            break;
        case 3: // Left: Middle Control
            chassis.setPose(-61, 16.2, 90);
            leftMiddleControl();
            break;
        case 4: // Right: Middle Control
            chassis.setPose(-61, -16.2, 90);
            rightMiddleControl();
            break;
        default:
            break;
    }
}

void scoreTop() {
    hoodBool=true;
    hood.set_value(HIGH);
    int speedForTop = 127;
    if(selectedAuton == 1){
        speedForTop = 100;
    }
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
    armMotor.move_absolute(620, speedForTop);
    pros::delay(300);
    armMotor.move_absolute(15, 40);
}
void scoreMiddle() {
    hoodBool=true;
    hood.set_value(HIGH);
    int speedForMiddle = 50;
    if(selectedAuton == 1){
        speedForMiddle = 40;
    }
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
    armMotor.move_absolute(610, speedForMiddle);
    pros::delay(500);
    armMotor.move_absolute(15, 40);
}
void park(){
    armMotor.move_absolute(100, 127);
}