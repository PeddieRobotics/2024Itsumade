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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    driveMotor.setVelocityPIDValues(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA,
        ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD,
        ModuleConstants.kTurningFF);

    driveMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    steerMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    // driveMotor.setBrake();
    // steerMotor.setBrake();

    // driveMotor.setClosedLoopRampRate(0.05);
    steerMotor.setClosedLoopRampRate(0.05);

    driveMotor.setPosition(0);
    steerMotor.setPosition(0);

    // Steer Encoder Setup
    steerEncoder = new CANcoder(CANCoderId, canbusName);
    configureCANcoder();

    steerMotor.setFeedbackDevice(1, FeedbackSensorSourceValue.SyncCANcoder);

    driveMotor.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    steerMotor.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositonFactor);
  }

  public SwerveModuleState getState() {
    return state;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition(), new Rotation2d(steerEncoder.getPosition().getValueAsDouble()));
  }

  public void setDesiredState(SwerveModuleState desiredModuleState) {
    desiredState = desiredModuleState;

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredModuleState,
        new Rotation2d(steerEncoder.getPosition().getValueAsDouble()));

    double desiredVelocity = optimizedDesiredState.speedMetersPerSecond * DriveConstants.kDrivingMotorGearRatio
        / (2 * DriveConstants.kWheelRadius);
    double desiredAngle = desiredState.angle.getRadians();

    driveMotor.setVelocityWithFeedForward(desiredVelocity);
    steerMotor.setPositionWithFeedForward(desiredAngle);
  }

  public void stop() {
    driveMotor.setMotor(0);
    steerMotor.setMotor(0);
  }

  public double getCANCoderReading() {
    return Math.IEEEremainder(steerEncoder.getPosition().getValueAsDouble() * 2 * Math.PI, 2 * Math.PI);
  }

  public void resetCANCoder() {
    steerEncoder.getConfigurator().setPosition(0.0);
  }

  public void resetTurnEncoder() {
    steerMotor.setPosition(0);
  }

  public void resetDriveEncoder() {
    driveMotor.setPosition(0);
  }

  public void configureCANcoder() {
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    steerEncoder.getConfigurator().apply(canCoderConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void putSmartDashboard(){
    SmartDashboard.putNumber("turning p", 0);
    SmartDashboard.putNumber("turning i", 0);
    SmartDashboard.putNumber("turning d", 0);
    SmartDashboard.putNumber("turning ff", 0);
    SmartDashboard.putBoolean("use turn position pid", false);
    SmartDashboard.putNumber("turning setpoint", 0);

    SmartDashboard.putNumber("driving s", 0.05);
    SmartDashboard.putNumber("driving v", 0.12);
    SmartDashboard.putNumber("driving a", 0);
    SmartDashboard.putNumber("driving p", 0.1);
    SmartDashboard.putNumber("driving i", 0);
    SmartDashboard.putNumber("driving d", 0);
    SmartDashboard.putNumber("driving ff", 0);
    SmartDashboard.putBoolean("use drive velocity pid", false);
    SmartDashboard.putNumber("drive mps setpoint", 0);
  }

  public void updateSmartdashBoard(){
    SmartDashboard.putNumber("drive motor position", driveMotor.getPosition());
    SmartDashboard.putNumber("turn motor position", steerMotor.getPosition());
    SmartDashboard.putNumber("drive motor mps", driveMotor.getMPS());
    SmartDashboard.putNumber("cancoder position", getCANCoderReading());
    SmartDashboard.putNumber("drive motor temperature", driveMotor.getMotorTemperature());
    SmartDashboard.putNumber("drive motor current", driveMotor.getSupplyCurrent());
    SmartDashboard.putNumber("turning motor output current", steerMotor.getSupplyCurrent());
    SmartDashboard.putNumber("turn motor temperature", steerMotor.getMotorTemperature());

    if (SmartDashboard.getBoolean("use turn position pid", false)) {
      steerMotor.setPIDValues(SmartDashboard.getNumber("turning p", 0), SmartDashboard.getNumber("turning i", 0),
          SmartDashboard.getNumber("turning d", 0), SmartDashboard.getNumber("turning ff", 0));
      steerMotor.setPositionWithFeedForward(SmartDashboard.getNumber("turning setpoint", 0));
    }

    if (SmartDashboard.getBoolean("use drive velocity pid", false)) {
      driveMotor.setVelocityPIDValues(
        SmartDashboard.getNumber("driving s", 0),
        SmartDashboard.getNumber("driving v", 0),
        SmartDashboard.getNumber("driving a", 0),
        SmartDashboard.getNumber("driving p", 0),
        SmartDashboard.getNumber("driving i", 0),
        SmartDashboard.getNumber("driving d", 0),
        SmartDashboard.getNumber("driving ff", 0)
        );
      driveMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("drive mps setpoint", 0));
    }
  }

}
