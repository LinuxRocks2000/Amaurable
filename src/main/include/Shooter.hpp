#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>

#include <Motor.hpp>
#include <shooterValues.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Note: limelight has a horizontal field of view of 54 degrees and a vertical of 41. It reports signed values, so -27 horizontal is all the way to the left and -20.5 vertical is all the way up.
// Round up vertical half-coords to (-21, 21) so it covers everything, and add one to horizontal half-coordinates (-28, 28) for the same reason. Otherwise segfaults.

typedef uint16_t INTerator;
typedef uint32_t bigASS;

#define SHOOTER_PLANE_SIZE 1600//1800 // Number of encoder ticks total in the shooter range

#define ANGLE_RANGE_START_Y -22.58
#define ANGLE_RANGE_END_Y   5.44


void interpolate(const tracking_value* values, double pos, tracking_value* ret){ // C style memory safety, in that you pass the blame.
    tracking_value before {-1, -1, -1};
    tracking_value after {-1, -1, -1};
    INTerator i = 0;
    while (1){
        if (values[i].position > pos){
            after = values[i];
            break;
        }
        before = values[i];
        if (values[i].position == 2 && values[i].perc == 2 && values[i].angle == 2){
            std::cerr << "WARNING: The interpolation request is not within the target set." << std::endl;
            break;
        }
        i ++;
    }
    double totalDist = after.position - before.position;
    double relativePos = pos - before.position;
    double pA = relativePos/totalDist;
    double pB = 1 - pA;
    ret -> position = pos;
    frc::SmartDashboard::PutNumber("Before Angle", before.angle);
    frc::SmartDashboard::PutNumber("After Angle", after.angle);
    ret -> perc = before.perc * pB + after.perc * pA;
    ret -> angle = before.angle * pB + after.angle * pA;
    frc::SmartDashboard::PutNumber("PERC", pos);
}


double convertAngleToPercentage(double angle){
    return (angle - ANGLE_RANGE_START_Y) // Make it so furthest bottom/top (whichever one) is 0
            / (ANGLE_RANGE_END_Y - ANGLE_RANGE_START_Y) // Divide by the max range so it becomes a value between 1 and 0.
            ;
}


class LimelightValues {
public:
    std::shared_ptr<nt::NetworkTable> camera;
    double tx = 0;
    double ty = 0;
    bool hasTarget = false;
    double _x = 0;
    double _y = 0;
    double _z = 0;

    void update(double xSkew = 0, double ySkew = 0){
        tx = camera -> GetNumber("tx", 0.0) + xSkew;
        ty = camera -> GetNumber("ty", 0.0) + ySkew;
        frc::SmartDashboard::PutNumber("Target X", tx);
        frc::SmartDashboard::PutNumber("Skew X", xSkew);
        frc::SmartDashboard::PutNumber("Target Y", ty);
        frc::SmartDashboard::PutNumber("Skew Y", ySkew);
        hasTarget = camera -> GetNumber("tv", 0) == 1 ? true : false;
        frc::SmartDashboard::PutBoolean("Limelight Has Target", hasTarget);
        _x = asin(tx);
        _y = asin(ty);
    }

    double getX(double distance = 1){
        return _x * distance;
    }

    double getY(double distance = 1){
        return _y * distance;
    }
};


class Shooter{
public:
    Motor* pusher;
    Motor* horizontal;
    Motor* vertical;
    Motor* flywheel;
    frc::DigitalInput* shooterPhotoelectric;

    std::shared_ptr<nt::NetworkTable> limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    LimelightValues values {limelight};

    tracking_value* interpolationResult = new tracking_value;

    bool zero = true;

    float spinAround = 0;
    bool bonk = false;
    // One rotation is 3982. Let's assume range of one rotation, though it's gonna be less.
    const int startPos = -750;
    const int endPos = 1500;

    bool isTrackLimelight = false;

    bool ignoreTarget = false;

    bool pushie = false;

    bool enabled = true;

    bool fixedAngleMode = false;

    bool barfMode = false;

    double tySkew = 0;

    void trackHorizontal(float limit = 0.2){
        isTrackLimelight = false;
        if ((values.hasTarget && !ignoreTarget) || fixedAngleMode){
            double speed = -1 * values.tx/28;
            horizontal -> Set(-speed * limit);
            if (horizontal -> GetPosition() > endPos){
                spinAround = -1;
                ignoreTarget = true;
            }
            else if (horizontal -> GetPosition() < startPos){
                spinAround = 1;
                ignoreTarget = true;
            }
        }
        else{
            if (!values.hasTarget){
                ignoreTarget = false;
            }
            if (spinAround == 1){
                horizontal -> Set(limit);
                if (horizontal -> GetPosition() < startPos){
                    spinAround = 0;
                }
                return;
            }
            if (spinAround == -1){
                horizontal -> Set(-limit);
                if (horizontal -> GetPosition() > endPos){
                    spinAround = 0;
                }
                return;
            }
            spinAround = (bonk ? 1 : -1);
            bonk = !bonk;
        }
    }

    void trackVertical(){
        vertical -> SetPosPID(interpolationResult -> angle * SHOOTER_PLANE_SIZE + 200); // Dammit, it's pretty accurately wrong by 200 ticks so 200 ticks is what I shall offset it by.
        frc::SmartDashboard::PutNumber("vertical position", interpolationResult -> angle * SHOOTER_PLANE_SIZE);
        frc::SmartDashboard::PutNumber("vertical real position", vertical -> GetPosition());
    }

    void init(){
        flywheel -> setInverted(false);
        flywheel -> configureTalonFXPidSpeed();
        vertical -> setInverted(true);
        vertical -> ZeroEncoder();
        vertical -> configureTalonFXPidPosition();
    }

    void wheel(){
        flywheel -> SetSpeed(10000 * interpolationResult -> perc);
        //flywheel -> Set(-1);
        frc::SmartDashboard::PutNumber("Shooter Percentage", interpolationResult -> perc);
        frc::SmartDashboard::PutNumber("Shooter Req Speed", 10000 * interpolationResult -> perc);
        frc::SmartDashboard::PutNumber("Shooter Real Speed", flywheel -> GetVelocity());
    }

    bool hasStartedZero = false;

    void disable(){
        flywheel -> Set(0);
        vertical -> Set(0);
        horizontal -> Set(0);
        horizontal -> ZeroEncoder();
        vertical -> ZeroEncoder();
        flywheel -> ZeroEncoder();
        enabled = false;
    }

    void enable(){
        enabled = true;
    }

    void run(){
        if (!enabled){
            return;
        }
        if (zero){
            if (!hasStartedZero){
                hasStartedZero = true;
                horizontal -> Set(-0.1);
                horizontal -> ZeroEncoder();
            }
            if (!shooterPhotoelectric -> Get()){
                std::cout << "ZEROED" << std::endl;
                horizontal -> ZeroEncoder();
                horizontal -> Set(0);
                zero = false;
            }
            if (horizontal -> GetPosition() > 500){
                horizontal -> Set(0.1);
            }
            else if (horizontal -> GetPosition() < -500){
                horizontal -> Set(0);
                std::cout << "SHOOTER DISABLED" << std::endl;
                disable();
                zero = false;
            }
            return;
        }
        values.update(0, tySkew);
        interpolate(interpolationValues, convertAngleToPercentage(values.ty), interpolationResult); // limelight's borken
        /*interpolationResult -> position = 0;
        interpolationResult -> perc = 0.9;
        interpolationResult -> angle = tySkew * 6;*/
        if (fixedAngleMode){
            interpolationResult -> position = 0;
            values.tx = 0;
        }
        trackHorizontal(0.5);
        frc::SmartDashboard::PutNumber("swivvel pos", horizontal -> GetPosition());
        frc::SmartDashboard::PutNumber("angle", interpolationResult -> angle);
        frc::SmartDashboard::PutNumber("rawAngle", values.ty);

        if (verticalZeroTicks <= 0){
            trackVertical();
        }
        if (!revMode){
            interpolationResult -> perc = 0.1;
        }
        wheel();

        if (pushie){
            pushie = false;
            pusher -> Set(0.75);
        }
        else{
            pusher -> Set(0);
        }

        barfMode = false;
        revMode = false;

        verticalZeroTicks --;
        if (verticalZeroTicks > 0){
            vertical -> ZeroEncoder();
            vertical -> Set(0);
        }
    }

    bool isReady(){
        return fabs(flywheel -> GetVelocity() - 10000 * interpolationResult -> perc) <= 100;
    }

    void push(){
        pushie = true;
    }

    void barf(){
        barfMode = true;
    }

    bool revMode = false;

    void rev(){
        revMode = true;
    }

    long verticalZeroTicks = -1;

    void zeroVertical(){
        verticalZeroTicks = 500;
    }

    void toggleFixedAngleMode(){
        fixedAngleMode = !fixedAngleMode;
    }
};
