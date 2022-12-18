/* Talon FX motor; can be used as a BaseMotor. */
/* TalonSRXMotor but it controls falcons. */


#include <ctre/Phoenix.h> /* Requires Phoenix along with this vendordep */
#include <BaseMotor.hpp>


class TalonFXMotor : public BaseMotor{
    TalonFX* talon;
public:
    TalonFXMotor (int canID){
        talon = new TalonFX(canID);
    }

    void SetPercent(double speed){
        talon -> Set(ControlMode::PercentOutput, speed);
    }

    void SetP(double kP){
        talon -> Config_kP(0, kP);
    }

    void SetI(double kI){
        talon -> Config_kI(0, kI);
    }

    void SetD(double kD){
        talon -> Config_kD(0, kD);
    }

    void SetF(double kF){
        talon -> Config_kF(0, kF);
    }

    void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF = 0, double kNominalOR = 0){
        talon -> ConfigPeakOutputForward(kPeakOF);
        talon -> ConfigPeakOutputReverse(kPeakOR);
        talon -> ConfigNominalOutputForward(kNominalOF);
        talon -> ConfigNominalOutputReverse(kNominalOR);
    }

    void SetPositionPID(double position){
        talon -> Set(ControlMode::Position, position);
    }

    void SetSpeedPID(double speed){
        talon -> Set(ControlMode::Velocity, speed);
    }
};
