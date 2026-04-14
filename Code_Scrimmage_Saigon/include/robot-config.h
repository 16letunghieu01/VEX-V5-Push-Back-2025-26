using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Left1;
extern motor Left2;
extern motor Left3;
extern motor Right1;
extern motor Right2;
extern motor Right3;
extern inertial Inertial;
extern controller Controller1;
extern motor InsideRoller;
extern motor Intake1;
extern motor Intake2;
extern digital_out PistonA;
extern digital_out PistonB;
extern digital_out PistonH;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );