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
import frc.robot.utils.Constants;
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

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  private double moduleAngularOffset;

  public SwerveModule(String canbusName, int drivingCANId, int steeringCANId, int CANCoderId, double moduleAngularOffset) {
    this.drivingCANId = drivingCANId;
    this.steeringCANId = steeringCANId;
    this.CANCoderId = CANCoderId;


    driveMotor = new Kraken(drivingCANId, canbusName);
    steerMotor = new Kraken(steeringCANId, canbusName);

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setSupplyCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    steerMotor.setSupplyCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

    driveMotor.setBrake();
    steerMotor.setBrake();

    driveMotor.setClosedLoopRampRate(0.1);
    steerMotor.setClosedLoopRampRate(0.1);

    driveMotor.setEncoder(0);
    steerMotor.setEncoder(0);

    // Steer Encoder Setup
    steerEncoder = new CANcoder(CANCoderId, canbusName);
    this.moduleAngularOffset = moduleAngularOffset;
    configureCANcoder();
    // steerEncoder.setPosition(0);

    steerMotor.setContinuousOutput();
    steerMotor.setFeedbackDevice(CANCoderId, FeedbackSensorSourceValue.FusedCANcoder);

    driveMotor.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    steerMotor.setRotorToSensorRatio(ModuleConstants.kTurningMotorReduction);
    steerMotor.setSensorToMechanismRatio(1.0);

    driveMotor.setVelocityPIDValues(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA,
        ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF);
    steerMotor.setPIDValues(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD,
        ModuleConstants.kTurningFF);

    putSmartDashboard();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getMPS(), new Rotation2d(getCANCoderReading()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition() * ModuleConstants.kDrivingEncoderPostionFactor, new Rotation2d(getCANCoderReading()));
  }

  public double getRotations() {
    return driveMotor.getPosition();
  }

  public void setDesiredState(SwerveModuleState desiredModuleState) {
    desiredState = desiredModuleState;

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
        new Rotation2d(getCANCoderReading()));

    double desiredVelocity = optimizedDesiredState.speedMetersPerSecond * ModuleConstants.kDriveMotorReduction
        / (2 * DriveConstants.kWheelRadius);
    double desiredAngle = optimizedDesiredState.angle.getRadians() / (2 * Math.PI);

    SmartDashboard.putNumber(drivingCANId + " optimized desired velocity", desiredVelocity);
    SmartDashboard.putNumber(steeringCANId + " optimized desired position", desiredAngle * 2 * Math.PI);
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
    steerMotor.setEncoder(0);
  }

  public void resetDriveEncoder() {
    driveMotor.setEncoder(0);
  }

  public void configureCANcoder() {
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfiguration.MagnetSensor.MagnetOffset = -moduleAngularOffset/(2 * Math.PI);
    steerEncoder.getConfigurator().apply(canCoderConfiguration);
  }

  @Override
  public void periodic() {
    updateSmartdashBoard();
    // This method will be called once per scheduler run
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

    // SmartDashboard.putNumber(drivingCANId + " Drive Motor Current", driveMotor.getSupplyCurrent());

    SmartDashboard.putNumber(CANCoderId + " canCoder position (rad)", getCANCoderReading());
    // SmartDashboard.putNumber(drivingCANId + " driving speed (mps)", driveMotor.getMPS());
    // SmartDashboard.putNumber(CANCoderId + " steer motor raw position", steerMotor.getPosition());



    // if (SmartDashboard.getBoolean(drivingCANId + " Use PID Output", false)) {
    //   driveMotor.setVelocityWithFeedForward(SmartDashboard.getNumber(drivingCANId + " Drive Motor PID Output ", 0));
    //   steerMotor.setPositionWithFeedForward(SmartDashboard.getNumber(steeringCANId + " Steer Motor PID Output ", 0)/ (2 * Math.PI));

    //   steerMotor.setPIDValues(
    //     SmartDashboard.getNumber(steeringCANId + " turning p", 36),
    //     SmartDashboard.getNumber(steeringCANId + " turning i", 0),
    //     SmartDashboard.getNumber(steeringCANId + " turning d", 0),
    //     SmartDashboard.getNumber(steeringCANId + " turning ff", 0));
    // }

    // SmartDashboard.putNumber(drivingCANId + " drive motor position", driveMotor.getPosition() * ModuleConstants.kDrivingEncoderPostionFactor);
    // SmartDashboard.putNumber(steeringCANId + " turn motor position", steerMotor.getPosition() * 2* Math.PI);
    //SmartDashboard.putNumber("drive motor mps", driveMotor.getMPS());
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
