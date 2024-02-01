
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <string>
#include <sstream>
#include <algorithm>

#include "vex.h"

using namespace vex;
competition Competition;

// Brain should be defined by default
brain Brain;


// Robot configuration code.
motor LeftDriveSmart = motor(PORT1, ratio18_1, false);
motor RightDriveSmart = motor(PORT2, ratio18_1, true);
inertial DrivetrainInertial = inertial(PORT3);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 320, 320, 40, mm, 1);

motor Harvester = motor(PORT4, ratio18_1, false);

motor Catapult = motor(PORT5, ratio18_1, false);

motor Frills = motor(PORT6, ratio18_1, false);

triport ThreeWirePort = triport(PORT22);
digital_out Solenoid = digital_out(ThreeWirePort.A);

controller Controller1 = controller(primary);

void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;


// Include the V5 Library
#include "vex.h"
#include <math.h> 
#include "MiniPID.h"

//MiniPID pid = MiniPID(0,0,0); //Function cannot compile |undefined reference to MiniPID::MiniPID(double, double, double)
// Allows for easier use of the VEX Library
using namespace vex;
// https://www.vexforum.com/t/user-control-and-autonomous/106690/5

// Natural Functions

typedef struct{
  double current;
	double kP;
	double kI;
	double kD;
	double target;
	double error;
	double integral;
	double derivative;
	double lastError;
	double threshold;
	int   lastTime;
} pid;

pid sPID;

void TriggerHappy(int timems) 
{
  Catapult.spin(reverse);
  wait(timems, msec);
  Catapult.stop();
  return;
}

void LockIt(){
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);
}
void UnlockIt(){
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
}

//Proportional





void HyprTurn(double tarVal, double tarRange){ // bad code.
int harharhar;

double highRange = tarVal + tarRange;
double lowRange = tarVal - tarRange;

bool invalid = true;
  while (invalid){

    if(DrivetrainInertial.heading(degrees) > highRange){ // 
      harharhar = 1; // up
    }
    else if (DrivetrainInertial.heading(degrees) < lowRange){
      harharhar = -1; // down
    }
    else{
      harharhar = 0;
    }

    switch (harharhar){
    case 1:
      LeftDriveSmart.spin(reverse);
      RightDriveSmart.spin(forward);
      break;
    case -1:
      LeftDriveSmart.spin(forward);
      RightDriveSmart.spin(reverse);
      break;
    case 0:
      invalid = false;
      break;
    }
    double error;
    error = tarVal-DrivetrainInertial.heading(degrees); // if rot=170 y tar=20, err=-150 
  
    Controller1.Screen.print("error: %1 \n rot: %2 \n tar: %3", error, 
    DrivetrainInertial.rotation(degrees), tarVal);
    wait(20, msec);
  }
}

//Power ratio - convert degrees to volts.
double ratio = 0.1; // temp ratio val
void proportionalTurnR(int x){  //power ratio - convert degrees to volts
  double destination = x;
  double error;
  error = destination-DrivetrainInertial.rotation(degrees); // if rot=170 y tar=20, err=-150 
  
  Brain.Screen.print("error: %1 \n rot: %2 \n tar: %3", error, 
    DrivetrainInertial.rotation(degrees), destination);

  LeftDriveSmart.spin(forward, error*ratio, volt); // Lspin -150 volt
  RightDriveSmart.spin(reverse, error*ratio, volt); //Rspin -150 volt 
}
void proportionalTurnL(int x){  //power ratio - convert degrees to volts
  double destination = x;
  double error;
  error = destination-DrivetrainInertial.rotation(degrees);

  Brain.Screen.print("error: %1 \n rot: %2 \n tar: %3", error, 
    DrivetrainInertial.rotation(degrees), destination);
    
  LeftDriveSmart.spin(reverse, error*ratio, volt);
  RightDriveSmart.spin(forward, error*ratio, volt);
}



//without PID
void Forward(int x){
  Drivetrain.driveFor(forward, x, mm);
  return;
  
}

void MotorDrive(double x, double y){
  LeftDriveSmart.spinFor(x, degrees, false);
  RightDriveSmart.spinFor(y, degrees, true);
}

void TurnTo(int x){
  Drivetrain.turnToHeading(x, degrees);
  return;
}
//-------------------- Main Functions

void pre_auton(void){
  calibrateDrivetrain();
  Catapult.setVelocity(100, percent); // catapult shoot speed
  Catapult.setMaxTorque(100, percent); // catapult torque
  Catapult.setStopping(hold);
  Solenoid.set(false); 
  //Catapult.setPosition(-30, degrees); 
  Catapult.spin(reverse);
  wait(750, msec);
  Catapult.stop();
}

void autonomous(void){
  
  DrivetrainInertial.setHeading(0, degrees);
  Drivetrain.turnToHeading(270, degrees);
  HyprTurn(270, 5);
  LockIt();
  //proportionalTurnR(50);
  Forward(-60);

  //MotorDrive(-300,-172.8); //300degree movement, 172.8 degree movement
  // 360deg = 8pi inches.  x% of 8pi = target inch. x% of 360 = target deg

  TriggerHappy(61000); // Shoots 50,seconds

  //TurnTo(300); // turns via heading.
  //Forward(2200); // Goes forward after shooting. Implementation is flawed.
  UnlockIt();
}

void usercontrol(void){
   while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Harvester
      if (Controller1.ButtonL1.pressing()) {
        Harvester.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Harvester.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Harvester.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Catapult
      if (Controller1.ButtonR1.pressing()) {
        Catapult.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Catapult.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }

      // check the buttonX status to control Pneumatics
      if (Controller1.ButtonX.pressing()){
        Solenoid.set(true); // pneumatic flows
        wait(500, msec); // allows time for pneumatic to do its thing
      }

      // check buttonB status to control Pneumatics
      if (Controller1.ButtonB.pressing()){ 
        Solenoid.set(false); // stops flow
        wait(500, msec); // same as above.
      }

      if (Controller1.ButtonDown.pressing()){
        LockIt();
      }

      if (Controller1.ButtonUp.pressing()){
        UnlockIt();
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
}


int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  pre_auton();

  while(true){
    wait(100, msec);
  }
}
