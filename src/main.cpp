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
void armLift(int n);
void wiggle(int num);
void toggleHood();
void toggleMiddle();

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
pros::Rotation lever(15);
// create an imu on port 8
pros::Imu imu(16);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_encoder(14);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,-.5);
pros::Rotation horizontal_encoder(11);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);

pros::Motor intake (9, pros::MotorGears::blue);
pros::Motor armMotor(-13, pros::MotorGears::red);

pros::adi::DigitalOut scraper('h',LOW);
pros::adi::DigitalOut middleGoal('g', HIGH);
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
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP) 6.1
                                              0.084, // integral gain (kI)-.006 over .0.1
                                              16, // derivative gain (kD) 40
                                              5, // anti windup 5
                                              1, // small error range, in inches  1
                                              100, // small error range timeout, in milliseconds 100
                                              3, // large error range, in inches  3
                                              500, // large error range timeout, in milliseconds 500
                                              0 // maximum acceleration (slew)20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.5,   // proportional gain (kP) 1.6
                       0.053,   // integral gain (kI) .0055
                       28.4,  // derivative gain (kD)//10.7 w/ ki  27.3
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

class PIDController {
public:
    // Constructor to initialize gains
    PIDController(double kp, double ki, double kd) 
        : Kp_(kp), Ki_(ki), Kd_(kd), integral_(0), prevError_(0) {}

    // Function to compute the new control output
    double compute(double setpoint, double current_value, double dt) {
        // 1. Calculate the current error
        double error = setpoint - current_value;

        // 2. Proportional term
        double proportional = Kp_ * error;

        // 3. Integral term (accumulate error over time)
        integral_ += error * dt;
        // NOTE: Anti-windup clamping may be needed in real applications
        if(integral_ > 100) integral_ = 100;
        if(integral_ < -100) integral_ = -100;
        // 4. Derivative term (rate of change of error)
        double derivative = (error - prevError_) / dt;

        // 5. Calculate the total output
        double output = proportional + (Ki_ * integral_) + (Kd_ * derivative);

        // 6. Update the previous error for the next iteration
        prevError_ = error;

        // NOTE: Output limits may also be needed
        return output;
    }
    void reset() {
        integral_ = 0;
        prevError_ = 0;
    }
    void set_constants(double p, double i, double d){ Kp_=p; Ki_=i; Kd_=d; }
private:
    double Kp_, Ki_, Kd_;
    double integral_;
    double prevError_;
};


int selectedAuton = 0;
bool autonSelected = false;
const char* autonNames[] = {"User", "Skills", "SAWP (Long)", "Left: Mid Control", "Right: Mid Control"};
int leverArmPosition = 0;

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
enum LeverMode {
    PID_MODE,
    FULL_UP,
    MIDDLE_SCORE
};
PIDController pid(1, 0.1, .01); 
LeverMode leverMode = PID_MODE;

void initialize() {
    // armMotor.move(30);
    // pros::delay(100000);
	pros::lcd::initialize(); // initialize brain screen. Remove for robodash
    chassis.calibrate(); // calibrate sensors
      lever.reset_position();
    leverArmPosition = 0;
    // pros::lcd::register_btn1_cb(on_center_button);
    scraperBool = false;
    wingBool = false;
    middleGoalBool = true;
    hoodBool = false;
   
    pros::Task screenTask([&]() {
      while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(3, "arm Deg: %d", lever.get_position() / 100);
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
      }
    });
    double output = 0;
    double pos;
    pros::Task leverTask([&]() {
    while (true) {

    
        
        if (leverMode == PID_MODE) {
            pos = lever.get_position()/100.0;
            double error = leverArmPosition - pos;

            if (fabs(error) < 1) {
                output = 0;
                
            } else {
                output = pid.compute(leverArmPosition, pos, 0.02);

                double minPower = 17;

                if (fabs(output) < minPower) {
                    output = (output > 0) ? minPower : -minPower;
                }
            }

            if (pos < 0.5 && output < 0) {
                output = 0;
            }
        }
        else if (leverMode == FULL_UP) {
            output = 127;
        }
        else if (leverMode == MIDDLE_SCORE) {
            pos = lever.get_position()/100.0;
            if(pos <= 20){
                output=127;
            }else{
                output = 40;
            }
            
        }

        if (output > 127) output = 127;
        if (output < -127) output = -127;
        
        
        armMotor.move(output);

        pros::delay(20);
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
        pros::delay(1000);
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
    scraper.set_value(HIGH);
    
    //get in front of loader
    chassis.turnToPoint(-47, 48, 1000);
    chassis.moveToPoint(-47, 38, 1000, {.minSpeed = 50, .earlyExitRange=4}); 
    

    //get balls in loader
    intake.move(127);
    chassis.moveToPoint(-65, 45, 1000, {.minSpeed = 60});

    pros::delay(1000);

    //Go to score
    chassis.turnToPoint(-28, 45, 2000, {.forwards=false});
    chassis.moveToPoint(-29, 45, 2000, {.forwards = false, .earlyExitRange = 4}, false);
    scraper.set_value(LOW);
    intake.move(0);
    scoreTop();

    // Get 3 balls
    
    intake.move(127);
    hood.set_value(LOW);
    chassis.moveToPoint(-32, 29, 2000, { .minSpeed=30, .earlyExitRange=1}, false);
    middleGoal.set_value(LOW);
    pros::delay(500);

    // Go score in middle
    chassis.turnToHeading(315, 1000, {.direction=AngularDirection::CW_CLOCKWISE});
    chassis.turnToPoint(-15, 10, 300, {.forwards = false});
    chassis.moveToPoint(-15, 10, 2000, {.forwards = false});
    pros::delay(500);
    scraper.set_value(LOW);
    scoreMiddle();
    
    // Go wing
    intake.move(0);
    middleGoal.set_value(HIGH);
    chassis.moveToPoint(-25, 30, 2000, {.minSpeed = 50, .earlyExitRange = 2});
    
    
    chassis.moveToPoint(-9, 40, 2000, {.minSpeed = 50});
    chassis.turnToHeading(120, 1000, {}, false);
    pros::delay(100);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void rightMiddleControl() {
    chassis.setPose(-47, -14, 180);
    middleGoal.set_value(HIGH);
    scraper.set_value(HIGH);
    
    //get in front of loader
    chassis.turnToPoint(-47, -48, 1000);
    chassis.moveToPoint(-47, -38, 1000, {.minSpeed = 50, .earlyExitRange=4}); 
    

    //get balls in loader
    intake.move(127);
    chassis.moveToPoint(-65, -45, 1000, {.minSpeed = 60});

    //Go to score
    chassis.turnToPoint(-28, -45, 2000, {.forwards=false});
    chassis.moveToPoint(-29, -45, 2000, {.forwards = false, .earlyExitRange = 6}, false);
    scraper.set_value(LOW);
    intake.move(0);
    scoreTop();
    
    // Get 3 balls
    intake.move(127);
    // hood.set_value(LOW);
    chassis.moveToPoint(-29, -26, 2000, { .minSpeed=30, .earlyExitRange=1}, false);
    chassis.turnToPoint(-17, -13, 1000);

    // Go score in middle
    chassis.moveToPoint(-19, -15, 2000);
    pros::delay(500);
    chassis.turnToHeading(45, 1000);
    armLift(5);
    intake.move(-100);
    
    pros::delay(1200);    

    // Go wing
    intake.move(0);
    chassis.moveToPoint(-20, -39, 2000, {.forwards=false, .minSpeed=50, .earlyExitRange=4});
    wing.set_value(HIGH);
    chassis.moveToPoint(-10, -41, 2000, {.forwards=false, .minSpeed=50});
    pros::delay(400);
    wing.set_value(LOW);
    chassis.turnToHeading(260, 1000, {}, false);
    pros::delay(100);
    forwardOritentiation=false;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void longSAWP() { // place only on right side
    chassis.setPose(-45, 0, 0);
    intake.move(127);
    chassis.moveToPoint(-45, 3, 2000, {.minSpeed=70, .earlyExitRange=2}); //push other bot

    //get in front of loader
    middleGoal.set_value(HIGH);
    

    //get in front of loader
    chassis.turnToPoint(-45, -42, 1000, {.forwards=false});
    chassis.moveToPoint(-45, -42, 1000, {.forwards=false,.minSpeed = 50, .earlyExitRange=4}); 

    
    intake.move(127);
    chassis.turnToPoint(-66, -44, 1000, {.minSpeed=70, .earlyExitRange=10});
    scraper.set_value(HIGH);
    chassis.moveToPoint(-66, -44, 1000, {.minSpeed = 70});
    pros::delay(200);
    chassis.turnToPoint(-25, -47, 750, {.forwards = false});
    // pros::delay(1000);

    //Go to score
    chassis.moveToPoint(-23, -47, 1000, {.forwards = false, .earlyExitRange=5}, false);
    scraper.set_value(LOW);
    // intake.move(0);
    scoreTop();

    // Get 3 balls
    intake.move(127);
    hood.set_value(LOW);
    chassis.moveToPoint(-12, -20, 2000, { .minSpeed=50, .earlyExitRange=16}, false);
    toggleHood();
    toggleMiddle();

    // Second 3 balls
    chassis.turnToPoint(-24, 18, 2000, {.minSpeed=80, .earlyExitRange=10});
    chassis.moveToPoint(-24, 18, 2000, {.minSpeed=40, .earlyExitRange=3});
    pros::delay(800); // adjust
    scraper.set_value(HIGH);
    middleGoal.set_value(LOW);
    

    // Middle scoring part
    chassis.turnToPoint(-6,0, 1000, {.forwards = false, .minSpeed=30, .earlyExitRange=8});
    
    chassis.moveToPoint(-16, 14, 2000, {.forwards = false});
    
    
    
    chassis.turnToPoint(-46, 51, 1000, {.minSpeed=50, .earlyExitRange=3});
    pros::delay(100);
    //middle score needs to be specific to not get balls under --------------
    hoodBool=true;
    hood.set_value(HIGH);
    leverMode = FULL_UP;
    pros::delay(250);
    intake.move(-127);
    leverMode = MIDDLE_SCORE;
    pros::delay(200);
    leverArmPosition=0;
    leverMode = PID_MODE;
    //armLift function but needs to be specific--------------------------
    
    // Second loader approach
    chassis.moveToPoint(-46, 50, 750, {.minSpeed=50, .earlyExitRange=5});
    intake.move(127);
    toggleHood();

    // Second loader
    scraper.set_value(HIGH);
    chassis.moveToPoint(-67, 47, 1000, {.minSpeed = 80}, false);
    toggleMiddle();
    wiggle(1);
    // pros::delay(1000);

    // Second goal scoring
    chassis.turnToPoint(-30, 47, 2000, {.forwards = false});
    chassis.moveToPoint(-29, 47, 1000, {.forwards = false, .minSpeed=30, .earlyExitRange=3}, false);

    scoreTop();
    pros::delay(700);

}

void skills() {
    middleGoalBool = true;
    hoodBool=false;
    middleGoal.set_value(middleGoalBool);
    hood.set_value(hoodBool);


    int run = 0;
    selectedAuton = 1;
    chassis.setPose(-45, 0, 270);//--------------------parking balls---------------
    armLift(7);
    intake.move(127);
    chassis.moveToPoint(-57, 0, 5000, {.minSpeed=70});
    pros::delay(500);
    armMotor.move_absolute(-10, 40);//bottom pose
    wiggle(3);
    chassis.moveToPoint(-60, 0, 5000);
    wiggle(3);
    chassis.moveToPoint(-42, 0, 5000);
    chassis.turnToPoint(-30.5, -15, 5000);
    chassis.moveToPoint(-30.5, -15, 5000);//get one ball for 7

    // chassis.turnToPoint(-30.5, -15, 5000);-----------middle scoring bottom-------------
    chassis.moveToPoint(-24, -24, 5000);//push balls out of way hopefully
    chassis.turnToPoint(-13, -13, 5000);
    armLift(7);
    chassis.moveToPoint(-13, -13, 5000);//score bottom
    pros::delay(500);
    intake.move(-60);
    pros::delay(2000);
    intake.move(0);
 //----------------------------------------------------long goal-------------------------------------------
    chassis.turnToPoint(-47, -49, 5000);
    chassis.moveToPoint(-47, -49, 5000, {}, false);
    scraper.set_value(HIGH);
    
    chassis.turnToHeading(270, 2000); //get balls in loader
    intake.move(127);
    chassis.moveToPoint(-59, -49, 1000, {.minSpeed = 50});
    chassis.turnToPoint(-47, -49, 1000, {.forwards = false});
    wiggle(3);
    // pros::delay(2000);

    chassis.turnToPoint(-47, -49, 5000, {.forwards = false});//back up a bit
    chassis.moveToPoint(-47, -49, 5000, {.forwards = false});
    scraper.set_value(LOW);
    intake.move(0);

    chassis.turnToPoint(-36, -63, 1000);//get in position to go across
    chassis.moveToPoint(-36, -63, 1000, {}, true);
    
    
    chassis.turnToHeading(90, 1000); //go across
    chassis.turnToPoint(0, -63.5, 2000);
    chassis.moveToPoint(0, -63.5, 5000, {.minSpeed = 50});
    // chassis.turnToPoint(36, 63.5, 2000, {});
    chassis.moveToPoint(38, -63.5, 5000, {.minSpeed = 50});

    int yVal = -49;
    chassis.turnToPoint(38, yVal, 2000); // get in front of loader
    chassis.moveToPoint(38, yVal, 5000);

    chassis.turnToHeading(90, 1000); //score first goal
    chassis.turnToPoint(34, yVal, 2000, {.forwards = false});
    intake.move(127);
    chassis.moveToPoint(34, yVal, 5000, {.forwards = false});
    scraper.set_value(HIGH);
    pros::delay(500);
    scoreTop();
    toggleHood();
    
    chassis.turnToPoint(65, yVal, 5000);// getting balls in loader
    chassis.moveToPoint(65, yVal, 5000);
    chassis.turnToPoint(34, yVal, 10000, {.forwards = false, .minSpeed = 50});
    wiggle(5);
    // pros::delay(2000);

   
    chassis.moveToPoint(34, yVal, 5000, {.forwards = false}); //score more
    chassis.turnToPoint(45, yVal, 2000);
    scoreTop();
    pros::delay(500);
    toggleHood();
    chassis.moveToPoint(45, yVal, 5000);//come out a little bit?
    //-------------------------------second parking balls---------------------------
    chassis.turnToPoint(45, 0, 5000);
    chassis.moveToPoint(45, 0, 5000);
    chassis.turnToHeading(90, 1000);
    
    chassis.moveToPoint(57, 0, 5000, {.minSpeed=30});
    wiggle(3);
    chassis.moveToPoint(60, 0, 5000);
    wiggle(3);
    chassis.moveToPoint(42, 0, 5000);
    chassis.turnToPoint(30.5, 15, 5000);
    chassis.moveToPoint(30.5, 15, 5000);//get one ball for 7

    //---------------------------------middle scoring top--------------------------
    chassis.moveToPoint(24, 24, 5000);//push balls out of way hopefully
    chassis.turnToPoint(13, 13, 5000, {.forwards=false});
    toggleMiddle();
    chassis.moveToPoint(13, 13, 5000, {.forwards=false});//score bottom
    pros::delay(500);
    scoreMiddle();
    pros::delay(300);
    intake.move(127);
    pros::delay(300);
    scoreMiddle();
    pros::delay(300);
    toggleMiddle();
    toggleHood();
    //-------------------------------second loader---------------------------
    

    pros::delay(10000000);
    //------------------park-----------------

    chassis.setPose(-32, -47, 0); // reset to right side long goal
    chassis.moveToPose(-60, 0, 180, 5000);
    armLift(7);



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
    scraperBool = false;//should be up in beginning
    wingBool = false;//should be down in beginning
    middleGoalBool = true;//should be true in beginning
    hoodBool = true;//should be up in beginning
    selectedAuton = 0;
    bool leverIsUp = false;
    // tuningLateralPD();
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
        
        //------------------------------------intakeing and outaking----------------------
        if (controller.get_digital(DIGITAL_R2)) { // intake
            hood.set_value(LOW);
            hoodBool=false;
			intake.move(127);
            armLift(0);
        }else if (controller.get_digital(DIGITAL_R1)) { // outake
            int speedForBottom = -127;
            if(selectedAuton == 1){//skills
                speedForBottom = -40;
            }
          	intake.move(speedForBottom);
            armLift(7);
            // armMotor.move_absolute(95, 50);
        }else{
            intake.move(0);
        }
            //------------------------------------lever control----------------------
        if (controller.get_digital(DIGITAL_L2)) { // top score
            leverIsUp=true;
            hoodBool=true;
            hood.set_value(HIGH);
            leverMode = FULL_UP;
            wingBool=false;
            wing.set_value(wingBool);
            
        }else if (controller.get_digital(DIGITAL_L1)) { // middle score
            leverIsUp=true;
            hoodBool=true;
            hood.set_value(HIGH);
            leverMode = MIDDLE_SCORE;
		}else if (leverIsUp){
            pid.reset();
            leverMode = PID_MODE;
            leverArmPosition = 0;
            leverIsUp=false;
        }

        //-----------------------------------pnumatics--------------------------
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){ // hood
            hoodBool=!hoodBool;
            hood.set_value(hoodBool);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){ // lever arm up
            armLift(7);
        }
        if (controller.get_digital_new_press(DIGITAL_B)) { // scraper
          scraperBool = !scraperBool;
          scraper.set_value(scraperBool);
          wingBool=false;
          wing.set_value(wingBool);
        }
        if (controller.get_digital_new_press(DIGITAL_Y)) { // toggle wing
          wingBool = !wingBool;
          wing.set_value(wingBool);
          scraperBool = false;
          scraper.set_value(scraperBool);
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
            chassis.setPose(-47, 14, 0);
            skills();
            break;
        case 2: // SAWP (Long)
            chassis.setPose(-46, 0, 0);
            longSAWP();
            break;
        case 3: // Left: Middle Control
            leftMiddleControl();
            break;
        case 4: // Right: Middle Control
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
    if(selectedAuton == 1){//skills
        speedForTop = 100;
    }
    leverMode = FULL_UP;
    pros::delay(500);
    leverArmPosition=0;
    leverMode = PID_MODE;
}
void scoreMiddle() {
    hoodBool=true;
    hood.set_value(HIGH);
    int speedForMiddle = 50;
    if(selectedAuton == 1){//skills
        speedForMiddle = 40;
    }
    leverMode = MIDDLE_SCORE;
    pros::delay(1000);
    leverArmPosition=0;
    leverMode = PID_MODE;
}
void armLift(int n){
    leverMode = PID_MODE;
    leverArmPosition=n;
}
void wiggle(int num){
  int ogAngle = chassis.getPose().theta;
  for(int i =0; i < num; i++){
    chassis.turnToHeading(ogAngle + 4, 1000, {.minSpeed=90});
    chassis.turnToHeading(ogAngle - 4, 1000, {.minSpeed=90});
  }
  chassis.turnToHeading(ogAngle, 1000);

}
void toggleHood(){
  hoodBool = !hoodBool;
  hood.set_value(hoodBool);
}
void toggleMiddle(){
  middleGoalBool=!middleGoalBool;
  middleGoal.set_value(middleGoalBool);
}