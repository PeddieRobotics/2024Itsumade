// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final CANcoder steerEncoder;

  private final int drivingCANId;
  private final int steeringCANId;
  private final int CANCoderId;

  private final Kraken driveMotor;
  private final Kraken steerMotor;
  
  private SwerveModuleState state;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  public SwerveModule(String canbusName, int drivingCANId, int steeringCANId, int CANCoderId) {
    this.drivingCANId = drivingCANId;
    this.steeringCANId = steeringCANId;
    this.CANCoderId = CANCoderId;

    driveMotor = new Kraken(drivingCANId, canbusName);
    steerMotor = new Kraken(steeringCANId, canbusName);

    driveMotor.setPIDValues(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF);

    driveMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    steerMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    driveMotor.setBrake();
    steerMotor.setBrake();

    driveMotor.setClosedLoopRampRate(0.05);
    steerMotor.setClosedLoopRampRate(0.05);

    // Steer Encoder Setup
    steerEncoder = new CANcoder(CANCoderId);
  }

  public SwerveModuleState getState(){
    return state;
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      driveMotor.getPosition(), new Rotation2d(steerEncoder.getPosition().getValueAsDouble()));
  }

  public void setDesiredState(SwerveModuleState desiredModuleState){
    desiredState = desiredModuleState;

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredModuleState, new Rotation2d(steerEncoder.getPosition().getValueAsDouble()));

    double desiredVelocity = optimizedDesiredState.speedMetersPerSecond * DriveConstants.kDrivingMotorGearRatio / (2 * DriveConstants.kWheelRadius);
    double desiredAngle = desiredState.angle.getRadians();

    driveMotor.setVelocityWithFeedForward(desiredVelocity);
    steerMotor.setPositionWithFeedForward(desiredAngle);
  }

  public void stop(){
    driveMotor.setMotor(0);
    steerMotor.setMotor(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
