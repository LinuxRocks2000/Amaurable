// Constants for the entire project.
/* Check Robot_Graphic.txt for CAN id graphics. */

// Constants for motors

#ifndef CONSTANTS_H // Make sure it only really #includes once.
#define CONSTANTS_H

#define NEO_500_RPM        5676

#define PID_SPEED_ACCEPTABLE_ERROR 100
#define PID_POSITION_ACCEPTABLE_ERROR 1

#define UNIVERSAL_DEADBAND 0.1 // The universal deadband for controls, this helps with our high-sensitivity issue.


const double PID_kP = 0.00001/*0.00004*/, PID_kI = 0, PID_kD = 0/*0.00025*/, PID_kIz = 0, PID_kFF = /*0*/0.000175, PID_kMaxOutput = 1, PID_kMinOutput = -1;

#define SHOOTERSPEED_LOW  NEO_500_RPM * 0.4
#define SHOOTERSPEED_MID  NEO_500_RPM * 0.75
#define SHOOTERSPEED_HIGH NEO_500_RPM * 0.7


#define CLIMBERSWITCH 1

#define LIMELIGHT_HUB_ANGLE_FOR_SHOOTING 21

#define AUTO_MIDSTALK_LENGTH 34


// Andrew - These CAN ids? They don't conform to your suggested convention.
// Ha! Punk'd!
#define MOTOR_BACK_LEFT   4
#define MOTOR_BACK_RIGHT  2
#define MOTOR_FRONT_LEFT  3
#define MOTOR_FRONT_RIGHT 1

#define MOTOR_INTAKE 5
#define MOTOR_INDEX  6

#define MOTOR_SHOOTER_PUSHER     7
#define MOTOR_SHOOTER_HORIZONTAL 8
#define MOTOR_SHOOTER_VERTICAL   9
#define MOTOR_SHOOTER_FLYWHEEL   10

enum ButtonBoard {
    INTAKE_UP     = 3, // Intake stuff
    INTAKE_DOWN   = 4,
    INTAKE_MACRO  = 5,
    SHOOTER_MACRO = -1, // Shooter stuff
};
#endif
