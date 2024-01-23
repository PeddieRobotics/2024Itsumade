// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    driveMotor.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPostionFactor);
    driveMotor.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    steerMotor.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositonFactor);
    steerMotor.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    driveMotor.setVelocityPIDValues(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA, ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF);

    driveMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    steerMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    // driveMotor.setBrake();
    // steerMotor.setBrake();

    driveMotor.setClosedLoopRampRate(0.05);
    steerMotor.setClosedLoopRampRate(0.05);

    driveMotor.setPosition(0);
    steerMotor.setPosition(0);

    // Steer Encoder Setup
    steerEncoder = new CANcoder(CANCoderId, canbusName);
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    steerMotor.setFeedbackDevice();

    SmartDashboard.putNumber("turning p", 0);
    SmartDashboard.putNumber("turning i", 0);
    SmartDashboard.putNumber("turning d", 0);
    SmartDashboard.putNumber("turning ff", 0);
    SmartDashboard.putBoolean("use turn position pid", false);
    SmartDashboard.putNumber("turning setpoint", 0);
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

  public double getCANCoderReading(){
    return Math.IEEEremainder(steerEncoder.getPosition().getValueAsDouble() * 2 * Math.PI, 2 * Math.PI);
  }

  public void resetCANCoder(){
    steerEncoder.getConfigurator().setPosition(0.0);
  }

  public void resetTurnEncoder(){
    steerMotor.setPosition(0);
  }

  public void resetDriveEncoder(){
    driveMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("drive motor position", driveMotor.getPosition());
    SmartDashboard.putNumber("turn motor position", steerMotor.getPosition());
    SmartDashboard.putNumber("drive motor velocity", driveMotor.getVelocity());
    SmartDashboard.putNumber("cancoder position", getCANCoderReading());
    SmartDashboard.putNumber("turning motor output current", steerMotor.getSupplyCurrent());
    SmartDashboard.putNumber("turn motor temperature", steerMotor.getMotorTemperature());

    if(SmartDashboard.getBoolean("use turn position pid", false)){
      steerMotor.setPIDValues(SmartDashboard.getNumber("turning p", 0), SmartDashboard.getNumber("turning i", 0), SmartDashboard.getNumber("turning d", 0), SmartDashboard.getNumber("turning ff", 0));
      steerMotor.setPositionWithFeedForward(SmartDashboard.getNumber("turning setpoint", 0));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
