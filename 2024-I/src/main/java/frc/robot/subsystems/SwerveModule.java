// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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

    driveMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    steerMotor.setCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    // driveMotor.setBrake();
    // steerMotor.setBrake();

    driveMotor.setClosedLoopRampRate(0.1);
    steerMotor.setClosedLoopRampRate(0.1);

    driveMotor.setPosition(0);
    steerMotor.setPosition(0);

    // Steer Encoder Setup
    steerEncoder = new CANcoder(CANCoderId, canbusName);
    configureCANcoder();
    steerEncoder.setPosition(0);

    steerMotor.setContinuousOutput();
    steerMotor.setFeedbackDevice(CANCoderId, FeedbackSensorSourceValue.RemoteCANcoder);

    driveMotor.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // REMOTE CANCODER
    steerMotor.setPositionConversionFactor(1.0);

    // INTERNAL SENSOR
    // steerMotor.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositonFactor);

    // FUSED CANCODER
    // steerMotor.setRotorToSensorRatio(ModuleConstants.kTurningEncoderPositonFactor);
    // steerMotor.setPositionConversionFactor(1.0);

    driveMotor.setVelocityPIDValues(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA,
        ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD,
        ModuleConstants.kTurningFF);

    putSmartDashboard();

    SmartDashboard.putBoolean("play music", false);
  }

  public SwerveModuleState getState() {
    return state;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition(), new Rotation2d(getCANCoderReading()));
  }

  public double getRotations() {
    return driveMotor.getPosition();
  }

  public void setDesiredState(SwerveModuleState desiredModuleState) {
    desiredState = desiredModuleState;

    // SmartDashboard.putNumber(drivingCANId + " desired velocity", desiredState.speedMetersPerSecond);
    // SmartDashboard.putNumber(steeringCANId + " desired position", desiredState.angle.getRadians());

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
        new Rotation2d(getCANCoderReading()));

    double desiredVelocity = optimizedDesiredState.speedMetersPerSecond * ModuleConstants.kDriveMotorReduction
        / (2 * DriveConstants.kWheelRadius);
    double desiredAngle = optimizedDesiredState.angle.getRadians() / (2 * Math.PI);

    SmartDashboard.putNumber(drivingCANId + " optimized desired velocity", desiredVelocity);
    SmartDashboard.putNumber(steeringCANId + " optimized desired position", desiredAngle);
    SmartDashboard.putNumber(steeringCANId + " steering motor position", getCANCoderReading());
    
    driveMotor.setVelocityWithFeedForward(desiredVelocity);
    steerMotor.setPositionWithFeedForward(desiredAngle);
  }

  public void stop() {
    driveMotor.setMotor(0);
    steerMotor.setMotor(0);
  }

  public double getCANCoderReading() {
    return steerEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // * 2 * Math.PI
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
    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    steerEncoder.getConfigurator().apply(canCoderConfiguration);
  }

  @Override
  public void periodic() {
    updateSmartdashBoard();
    // This method will be called once per scheduler run
    driveMotor.playMusic(SmartDashboard.getBoolean("play music", false));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void putSmartDashboard() {
    // SmartDashboard.putNumber(drivingCANId + " Drive Motor Percent Output ", 0);
    // SmartDashboard.putNumber(steeringCANId + " Steer Motor Percent Output ", 0);
    // SmartDashboard.putBoolean(drivingCANId + " Use Percent Output", false);

    // SmartDashboard.putBoolean(drivingCANId + " Use PID Output", false);
    // SmartDashboard.putNumber(drivingCANId + " Drive Motor PID Output ", 0);
    // SmartDashboard.putNumber(steeringCANId + " Steer Motor PID Output ", 0);

    // SmartDashboard.putNumber(steeringCANId + " turning p", 0);
    // SmartDashboard.putNumber(steeringCANId + " turning i", 0);
    // SmartDashboard.putNumber(steeringCANId + " turning d", 0);
    // SmartDashboard.putNumber(steeringCANId + " turning ff", 0);

    // SmartDashboard.putBoolean(steeringCANId + " use turn position pid", false);
    // SmartDashboard.putNumber(steeringCANId + " turning setpoint", 0);

    // SmartDashboard.putNumber(drivingCANId + " driving s", 0.05);
    // SmartDashboard.putNumber(drivingCANId + " driving v", 0.12);
    // SmartDashboard.putNumber(drivingCANId + " driving a", 0);
    // SmartDashboard.putNumber(drivingCANId + " driving p", 0.1);
    // SmartDashboard.putNumber(drivingCANId + " driving i", 0);
    // SmartDashboard.putNumber(drivingCANId + " driving d", 0);
    // SmartDashboard.putNumber(drivingCANId + " driving ff", 0);
    // SmartDashboard.putBoolean(drivingCANId + " use drive velocity pid", false);
    // SmartDashboard.putNumber(drivingCANId + " drive mps setpoint", 0);
  }

  public void updateSmartdashBoard() {
    // if(SmartDashboard.getBoolean(drivingCANId + " Use Percent Output", false)){s
    // driveMotor.setMotor(SmartDashboard.getNumber(drivingCANId + " Drive Motor
    // Percent Output ", 0));
    // steerMotor.setMotor(SmartDashboard.getNumber(steeringCANId + " Steer Motor
    // Percent Output ", 0));
    // }

    SmartDashboard.putNumber(CANCoderId + " canCoder position (rad)", getCANCoderReading());
    SmartDashboard.putNumber(drivingCANId + " driving speed (mps)", driveMotor.getRPS());
    // SmartDashboard.putNumber(CANCoderId + " steer motor raw position", steerMotor.getPosition());



    // if (SmartDashboard.getBoolean(drivingCANId + " Use PID Output", false)) {
    //   driveMotor.setVelocityWithFeedForward(SmartDashboard.getNumber(drivingCANId + " Drive Motor PID Output ", 0));
    //   steerMotor.setPositionWithFeedForward(SmartDashboard.getNumber(steeringCANId + " Steer Motor PID Output ", 0)/ (2 * Math.PI));

    //   steerMotor.setPIDValues(
    //     SmartDashboard.getNumber(steeringCANId + " turning p", 0),
    //     SmartDashboard.getNumber(steeringCANId + " turning i", 0),
    //     SmartDashboard.getNumber(steeringCANId + " turning d", 0),
    //     SmartDashboard.getNumber(steeringCANId + " turning ff", 0));
    // }

    // SmartDashboard.putNumber("drive motor position", driveMotor.getPosition());
    // SmartDashboard.putNumber("turn motor position", steerMotor.getPosition());
    // SmartDashboard.putNumber("drive motor mps", driveMotor.getMPS());
    // SmartDashboard.putNumber(CANCoderId + " cancoder position",
    // getCANCoderReading());
    // SmartDashboard.putNumber("drive motor temperature",
    // driveMotor.getMotorTemperature());
    // SmartDashboard.putNumber("drive motor current",
    // driveMotor.getSupplyCurrent());
    // SmartDashboard.putNumber("turning motor output current",
    // steerMotor.getSupplyCurrent());
    // SmartDashboard.putNumber("turn motor temperature",
    // steerMotor.getMotorTemperature());

    // if (SmartDashboard.getBoolean("use turn position pid", false)) {
    // steerMotor.setPIDValues(SmartDashboard.getNumber("turning p", 0),
    // SmartDashboard.getNumber("turning i", 0),
    // SmartDashboard.getNumber("turning d", 0), SmartDashboard.getNumber("turning
    // ff", 0));
    // steerMotor.setPositionWithFeedForward(SmartDashboard.getNumber("turning
    // setpoint", 0));
    // }

    // if (SmartDashboard.getBoolean("use drive velocity pid", false)) {
    // driveMotor.setVelocityPIDValues(
    // SmartDashboard.getNumber("driving s", 0),
    // SmartDashboard.getNumber("driving v", 0),
    // SmartDashboard.getNumber("driving a", 0),
    // SmartDashboard.getNumber("driving p", 0),
    // SmartDashboard.getNumber("driving i", 0),
    // SmartDashboard.getNumber("driving d", 0),
    // SmartDashboard.getNumber("driving ff", 0)
    // );
    // driveMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("drive mps
    // setpoint", 0));
    // }
  }

}
