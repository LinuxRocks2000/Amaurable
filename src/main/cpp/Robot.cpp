// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ModularRobot.hpp"
#include <Motor.hpp>
#include <Drive.hpp>
#include <Intake.hpp>
#include <Indexer.hpp>
#include <ButtonTracker.hpp>
#include <Shooter.hpp>

#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/DigitalInput.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

typedef rev::CANSparkMax SparkMax;

#define INTAKE_INDEX_READY 2
#define INTAKE_STATE_IDLE  0

void TODO(std::string reason){
    std::cerr << "ERROR: todo " + reason + "." << std::endl;
}


class Robot : public ModularRobot {
    SparkMax backLeftSpark      {MOTOR_BACK_LEFT,        rev::CANSparkMax::MotorType::kBrushless};
    SparkMax backRightSpark     {MOTOR_BACK_RIGHT,       rev::CANSparkMax::MotorType::kBrushless};
    SparkMax frontLeftSpark     {MOTOR_FRONT_LEFT,       rev::CANSparkMax::MotorType::kBrushless};
    SparkMax frontRightSpark    {MOTOR_FRONT_RIGHT,      rev::CANSparkMax::MotorType::kBrushless};
    SparkMax indexerSpark       {MOTOR_INDEX,            rev::CANSparkMax::MotorType::kBrushless};
    //SparkMax flywheelSpark      {MOTOR_SHOOTER_FLYWHEEL, rev::CANSparkMax::MotorType::kBrushless};
    //SparkMax shooterVertical    {MOTOR_SHOOTER_VERTICAL, rev::CANSparkMax::MotorType::kBrushless};

    Motor* backLeft =   Motor::CreateMotor(&backLeftSpark);
    Motor* backRight =  Motor::CreateMotor(&backRightSpark);
    Motor* frontLeft =  Motor::CreateMotor(&frontLeftSpark);
    Motor* frontRight = Motor::CreateMotor(&frontRightSpark);
    Motor* flywheel =   Motor::CreateMotor(new TalonFX(MOTOR_SHOOTER_FLYWHEEL));

    Drivetrain drive {backLeft, backRight, frontLeft, frontRight};

    frc::DigitalInput intakePhotoelectric {9};
    Motor* intakeMotor = Motor::CreateMotor(new VictorSPX {MOTOR_INTAKE});
    frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 0, 1};
    Intake taker {&intakePhotoelectric, intakeMotor, &intakeSolenoid};

    Motor* indexMotor = Motor::CreateMotor(&indexerSpark);
    Indexer indie {indexMotor};

    frc::Joystick joy {5};
    ButtonTracker buttons {&joy, 1, 12};

    frc::XboxController xbox {4};

    frc::GenericHID buttonBoardHID {3};
    ButtonTracker buttonBoard {&buttonBoardHID, 1, 13};

    frc::Compressor compressor {frc::PneumaticsModuleType::CTREPCM};

    Motor* pusherMotor = Motor::CreateMotor(new VictorSPX {MOTOR_SHOOTER_PUSHER});
    Motor* shooterHorizontal = Motor::CreateMotor(new TalonSRX {MOTOR_SHOOTER_HORIZONTAL});
    Motor* shooterVertical = Motor::CreateMotor(new TalonFX {MOTOR_SHOOTER_VERTICAL});
    frc::DigitalInput shooterPhotoelectric{8};
    Shooter shooter {pusherMotor, shooterHorizontal, shooterVertical, flywheel, &shooterPhotoelectric};

    void Init(){
        setData("Amaurable", "Firestorm Robotics", 6341);
        frontRight -> setInverted(true);
        backRight -> setInverted(true);
        compressor.EnableDigital();
        shooter.init();
    }

    void joystickMode(){
        drive.turn(joy.GetX());
        drive.forward(joy.GetY());
        drive.limit((joy.GetThrottle() + 1)/2);

        buttons.test();
        if (buttons.GetReleased(2)){
            taker.intake();
        }
    }

    bool down = false;

    void barf(){
        indie.barf();
        taker.barf();
        shooter.barf();
    }

    void xboxMode(){ // Generic tank Drive
        drive.left(xbox.GetRawAxis(1));
        drive.right(xbox.GetRawAxis(5));
        // Swaous Drive
        /*drive.forward(xbox.GetRawAxis(1));
        drive.turn(xbox.GetRawAxis(0));
        drive.limit(xbox.GetRawAxis(5));*/
        // Introverted drive
        /*drive.left(xbox.GetRawAxis(0));
        drive.left(-xbox.GetRawAxis(4));
        drive.right(xbox.GetRawAxis(1));
        drive.right(-xbox.GetRawAxis(5));*/
        // Max's drive (the epicist drive)
        /*drive.left(xbox.GetRawAxis(3));
        drive.right(xbox.GetRawAxis(2));
        drive.limit(.25);*/
        // Angle drive
        //drive.left(xbox.GetRawAxis(0));
        //drive.right(xbox.GetRawAxis(1));


        if (xbox.GetBButton()){
            barf();
        }
        if (xbox.GetXButton()){
            shooter.rev();
            if (shooter.isReady()){
                indie.up();
                shooter.push();
            }
        }
        if (xbox.GetAButtonPressed()){
            taker.intake();
            taker.down();
        }
        if (xbox.GetAButton()){
            if (taker.state == INTAKE_STATE_IDLE){
                taker.intake();
            }
        }
        if (xbox.GetAButtonReleased()){
            taker.cancel();
            taker.up();
        }

    }

    void buttonBoardMode(){
        drive.limit((buttonBoardHID.GetRawAxis(0) - 1)/2);
        buttonBoard.test();
        shooter.tySkew = buttonBoardHID.GetRawAxis(2) * 4;
        if (buttonBoard.GetReleased(4)){
            taker.up();
        }
        if (buttonBoard.GetReleased(3)){
            taker.down();
        }
        if (buttonBoard.GetReleased(11)){
            shooter.toggleFixedAngleMode();
        }

        if (buttonBoard.GetReleased(5)){
            taker.intake();
        }
        if (buttonBoard.GetReleased(9)){
            if (shooter.enabled){
                shooter.disable();
            }
            else{
                shooter.enable();
            }
        }
        if (buttonBoard.GetReleased(13)){
            shooter.zeroVertical();
        }
    }

    void TeleopInit(){
        taker.resetPneumatics();
        shooterVertical -> ZeroEncoder();
    }

    void count(){ // Keep a running balls count
        //TODO ("implement balls count");
    }

    void TeleopLoop(){
        if (xbox.IsConnected()){
            xboxMode();
        }
        if (joy.IsConnected()){
            joystickMode();
        }
        if (buttonBoardHID.IsConnected()){
            buttonBoardMode();
        }
        count();
        short intakeState = taker.run();
        if (intakeState == INTAKE_INDEX_READY){
            indie.index();
        }
        indie.run();

        drive.apply();

        shooter.run();
    }

    // Get current time: . It returns a weird unit so cast to long.
    // Run drivetrain: read Drive.hpp. The Drivetrain object is named "drive"

    void BeginAutonomous(){
        //long frc::Timer::GetFPGATimestamp() = currentTime

    }

    void AutonomousLoop(){

        //drive.forward(-3);

        //}

    }

    void TestLoop(){
        frc::SmartDashboard::PutNumber("SWIVVEL ENCODER", shooterHorizontal -> GetPosition());
    }

    void BeginTest(){
        shooterHorizontal -> ZeroEncoder();
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
