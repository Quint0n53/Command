// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <units/voltage.h>
#include <units/voltage.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"

#include "Constants.h"
using namespace frc;
using namespace units;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void ArcadeDrive(double fwd,double rot);

  void TankDriveVolts(volt_t left, volt_t right);

  void ResetEncoders();

  void ResetEncoders();

  double GetAverageEncoderDistance();

  // TalonFX & GetLeftEncoder();

  // TalonFX & GetRightEncoder();

  void SetMaxOutput(double maxOutput);

  degree_t GetHeading();

  double GetTurnRate();

  Pose2d GetPose();

  DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void ResetOdometry(Pose2d pose);

 private:
  WPI_TalonFX * motorTopLeft = new WPI_TalonFX(1);
  WPI_TalonFX * motorFrontLeft = new WPI_TalonFX(2);
  WPI_TalonFX * motorBackLeft = new WPI_TalonFX(3);
  WPI_TalonFX * motorTopRight = new WPI_TalonFX(4);
  WPI_TalonFX * motorFrontRight = new WPI_TalonFX(5);
  WPI_TalonFX * motorBackRight = new WPI_TalonFX(6);
  

  AHRS * gyro = new AHRS(SPI::Port::kMXP);

  MotorControllerGroup left{*motorTopLeft,*motorBackLeft,*motorFrontLeft};
  MotorControllerGroup right{*motorTopRight,*motorBackRight,*motorFrontRight};

  DifferentialDrive drive{left,right};

  DifferentialDriveOdometry m_odometry;
};
