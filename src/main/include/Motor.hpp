// Unified motor struct for Talons and Sparks. Will have PID control.

#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <functions.hpp>
#include <constants.h>

#ifndef Motor_hpp_INCLUDED // If it hasn't been included in this project before, pass this line.
#define Motor_hpp_INCLUDED // Flag it as included so next time somebody does #include <util/Motor.hpp>, it uses the old one instead.

#define MOTOR_TYPE_NONE    0
#define MOTOR_TYPE_TALON   1
#define MOTOR_TYPE_SPARK   2
#define MOTOR_TYPE_VICTOR  3
#define MOTOR_TYPE_TALONFX 4

#define MOTOR_PID_MODE_NONE  0
#define MOTOR_PID_MODE_SPEED 1
#define MOTOR_PID_MODE_POS   2

struct Motor {
    TalonSRX* talon;
    VictorSPX* victor;
    TalonFX* talonFx;
    rev::CANSparkMax* spark;
    rev::SparkMaxPIDController sparkPID;
    rev::SparkMaxRelativeEncoder sparkEncoder;
    uint8_t type = MOTOR_TYPE_NONE;

    double PIDPos = 0;
    double PIDSpeed = 0;

    uint8_t opMode = MOTOR_PID_MODE_NONE;

    bool zero = false;

    bool inverse = false;

    double zeroPoint = 0;

    static Motor* CreateMotor(TalonSRX* controller) {
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> talon = controller;
        ret -> type = MOTOR_TYPE_TALON;
        TalonSRXConfiguration config;
        config.peakCurrentLimit = 30;
        config.peakCurrentDuration = 1000;
        config.continuousCurrentLimit = 30;
        controller -> ConfigAllSettings(config);
        controller -> ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 30);
        return ret;
    }

    static Motor* CreateMotor(rev::CANSparkMax* controller) {
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> spark = controller;
        ret -> type = MOTOR_TYPE_SPARK;
        ret -> sparkPID = controller -> GetPIDController();
        ret -> sparkEncoder = controller -> GetEncoder();
        util::setPIDPresets(ret -> sparkPID);
        return ret;
    }

    static Motor* CreateMotor(VictorSPX* controller){
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> victor = controller;
        ret -> type = MOTOR_TYPE_VICTOR;
        return ret;
    }

    static Motor* CreateMotor(TalonFX* controller){
        Motor* ret = (Motor*)malloc(sizeof(Motor));
        memset(ret, 0, sizeof(Motor));
        ret -> talonFx = controller;
        ret -> type = MOTOR_TYPE_TALONFX;
        return ret;
    }

    void configureTalonFXPidSpeed(){
        talonFx -> ConfigFactoryDefault();
        talonFx -> ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, 0, 30);

        talonFx -> ConfigNeutralDeadband(0.001);

        talonFx -> ConfigNominalOutputForward(0, 30);
        talonFx -> ConfigNominalOutputReverse(0, 30);
        talonFx -> ConfigPeakOutputForward(1, 30);
        talonFx -> ConfigPeakOutputReverse(1, 30);
        talonFx -> Config_kP(0, 0.22, 30);
        talonFx -> Config_kI(0, 0.001, 30);
        talonFx -> Config_kD(0, 5, 30);
        talonFx -> Config_kF(0, 1023.0/20660.0, 30);
    }

    void configureTalonFXPidPosition(){
        talonFx -> ConfigFactoryDefault();
        talonFx -> ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, 0, 30);

        talonFx -> ConfigNeutralDeadband(0.001);

        talonFx -> ConfigNominalOutputForward(0, 30);
        talonFx -> ConfigNominalOutputReverse(0, 30);
        talonFx -> ConfigPeakOutputForward(1, 30);
        talonFx -> ConfigPeakOutputReverse(1, 30);
        talonFx -> Config_kP(0, 0.112, 30);
        talonFx -> Config_kI(0, 0, 30);
        talonFx -> Config_kD(0, 0, 30);
        talonFx -> Config_kF(0, 0, 30);
        talonFx -> ConfigAllowableClosedloopError(0, 0, 30);
    }

    void _drivePercent(double amount) {
        if (type == MOTOR_TYPE_TALON) {
            talon -> Set(ControlMode::PercentOutput, amount);
        }
        else if (type == MOTOR_TYPE_SPARK){
            spark -> Set(amount);
        }
        else if (type == MOTOR_TYPE_VICTOR){
            victor -> Set(ControlMode::PercentOutput, amount);
        }
        else if (type == MOTOR_TYPE_TALONFX){
            talonFx -> Set(TalonFXControlMode::PercentOutput, amount);
        }
    }

    void setInverted(bool val){
        inverse = val;
        if (type == MOTOR_TYPE_TALONFX){
            talonFx -> SetInverted(val);
            inverse = !val;
        }
    }

    void _drivePIDTo(long pos){
        if (type == MOTOR_TYPE_TALON){

        }
        else if (type == MOTOR_TYPE_SPARK){
            sparkPID.SetReference(pos, rev::CANSparkMax::ControlType::kPosition);
        }
        PIDPos = pos;
    }

    bool _driveCustomPIDTo(double pos, double reduction = 0.2){
        double dist = pos - GetPosition();
        double distPerc = util::sigmoid(10, dist);
        PIDPos = pos;
        Set(distPerc * reduction);
        if (fabs(GetPosition() - pos) < PID_POSITION_ACCEPTABLE_ERROR){
            return true;
        }
        return false;
    }

    void _drivePIDSpeed(long speed){
        if (type == MOTOR_TYPE_TALON){

        }
        else if (type == MOTOR_TYPE_TALONFX){
            talonFx -> Set(TalonFXControlMode::Velocity, speed);
        }
        else if (type == MOTOR_TYPE_SPARK) {
            sparkPID.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
        }
        PIDSpeed = speed;
    }

    void Set(double amount) {
        if (inverse){
            amount *= -1;
        }
        _drivePercent(amount);
    }

    bool SetPos(double pos, double lim = 0.2) {
        //_drivePIDTo(pos);
        return _driveCustomPIDTo(pos, lim);
    }

    void SetPosPID(double pos){
        if (type == MOTOR_TYPE_TALONFX){
            talonFx -> Set(TalonFXControlMode::Position, pos);
        }
    }

    double GetPosition(){
        if (type == MOTOR_TYPE_SPARK){
            return sparkEncoder.GetPosition() - zeroPoint;
        }
        else if (type == MOTOR_TYPE_TALON){
            return talon -> GetSelectedSensorPosition() - zeroPoint;
        }
        else if (type == MOTOR_TYPE_TALONFX){
            return talonFx -> GetSelectedSensorPosition() - zeroPoint;
        }
        return 0; // Just in case.
    }

    double GetVelocity(){
        if (type == MOTOR_TYPE_SPARK){
            return sparkEncoder.GetVelocity();
        }
        else if (type == MOTOR_TYPE_TALONFX){
            return talonFx -> GetSelectedSensorVelocity();
        }
        return 0;
    }

    void ZeroEncoder(){
        if (type == MOTOR_TYPE_SPARK){
            sparkEncoder.SetPosition(0);
        }
        else if (type == MOTOR_TYPE_TALON){
            talon -> SetSelectedSensorPosition(0);
        }
        else if (type == MOTOR_TYPE_TALONFX){
            talonFx -> SetSelectedSensorPosition(0);
        }
        //zeroPoint = GetPosition();
        zero = true;
    }

    bool IsZeroed(){
        if (zero){
            if (fabs(GetPosition()) < 0.1){ // 0.1 encoder drift.
                zero = false;
                return true;
            }
            else{
                return false;
            }
        }
        return true;
    }

    bool SetSpeed(long speed){
        _drivePIDSpeed(speed);
        if (abs(GetVelocity() - speed) < PID_SPEED_ACCEPTABLE_ERROR) {
            return true;
        }
        return false;
    }
};
#endif
