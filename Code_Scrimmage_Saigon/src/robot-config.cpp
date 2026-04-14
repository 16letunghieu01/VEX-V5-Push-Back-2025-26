#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Left1 = motor(PORT8, ratio18_1, true);
motor Left2 = motor(PORT7, ratio18_1, true);
motor Left3 = motor(PORT10, ratio18_1, true);
motor Right1 = motor(PORT4, ratio18_1, false);
motor Right2 = motor(PORT5, ratio18_1, false);
motor Right3 = motor(PORT2, ratio18_1, false);
inertial Inertial = inertial(PORT6);
controller Controller1 = controller(primary);
motor InsideRoller = motor(PORT3, ratio18_1, true);
motor Intake1 = motor(PORT9, ratio18_1, true);
motor Intake2 = motor(PORT1, ratio18_1, false);
digital_out PistonA = digital_out(Brain.ThreeWirePort.A);
digital_out PistonB = digital_out(Brain.ThreeWirePort.B);
digital_out PistonH = digital_out(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}