#include <Motor.hpp>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>

#define INTAKE_SPEED -0.75


class Intake{
public: // Andrew, I'm doing this to annoy you.
    frc::DigitalInput* photoelectric;
    Motor* motor;
    frc::DoubleSolenoid* solenoid;
    short state = 0;
    // 0 = static, 1 = running waiting for ball, 2 = ball in sensor, running till it's out.
    bool pos = true;

    void intake(){
        if (!pos){
            return;
        }
        if (state == 1){
            state = 0;
            return;
        }
        state = 1;
    }

    void resetPneumatics(){
        solenoid -> Set(frc::DoubleSolenoid::Value::kOff);
    }

    char run(){
        if
        (state == 0){
             motor -> Set(0);
        } else if
        (state == 1){
            motor -> Set(INTAKE_SPEED);
            if (photoelectric -> Get()){
                state = 2;
            }
        } else if
        (state == 2){ // When this is returned, the robot main should signal to index a new ball so the indexer works with it.
            motor -> Set(INTAKE_SPEED);
            if (!photoelectric -> Get()){
                state = 0;
            }
        } else if
        (state == 3){
            motor -> Set(-INTAKE_SPEED);
            state = 0;
        }
        return state;
    }

    void barf(){
        state = 3;
    }

    void down(){
        solenoid -> Set(frc::DoubleSolenoid::Value::kReverse);
        pos = false;
    }

    void up(){
        solenoid -> Set(frc::DoubleSolenoid::Value::kForward);
        pos = true;
    }

    void cancel(){
        state = 0;
    }
};
