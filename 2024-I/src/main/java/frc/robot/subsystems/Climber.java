// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Climber climber;
  private Kraken rightClimber, leftClimber;
  private DigitalInput climberSensor;

  public Climber() {
    rightClimber = new Kraken(RobotMap.CLIMBER_RIGHT_MOTOR, RobotMap.CANIVORE_NAME);
    leftClimber = new Kraken(RobotMap.CLIMBER_LEFT_MOTOR, RobotMap.CANIVORE_NAME);

    rightClimber.resetEncoder();
    leftClimber.resetEncoder();

    rightClimber.setSoftLimits(true, ClimberConstants.kClimberForwardSoftLimit, ClimberConstants.kClimberReverseSoftLimit);
    leftClimber.setSoftLimits(true, ClimberConstants.kClimberForwardSoftLimit, ClimberConstants.kClimberReverseSoftLimit);

    rightClimber.setSupplyCurrentLimit(ClimberConstants.kClimberCurrentLimit);
    leftClimber.setSupplyCurrentLimit(ClimberConstants.kClimberCurrentLimit);

    rightClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);
    leftClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);

    rightClimber.setPIDValues(ClimberConstants.kClimberP,
        ClimberConstants.kClimberI, ClimberConstants.kClimberD,
        ClimberConstants.kClimberFF);
    leftClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD,
        ClimberConstants.kClimberFF);

    rightClimber.setBrake();
    leftClimber.setBrake();

    // SmartDashboard.putBoolean("Manual Climber Control", false);
    // SmartDashboard.putBoolean("Climber PID Tuning", false);
    // SmartDashboard.putNumber("Climber P Value", 0);
    // SmartDashboard.putNumber("Climber I Value", 0);
    // SmartDashboard.putNumber("Climber D Value", 0);
    // SmartDashboard.putNumber("Climber FF Value", 0);

    // SmartDashboard.putNumber("Climber Angle Setpoint", 0);

  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }

  public void retractClimber() {
    if (!leftArmRetracted()) {
      runLeftMotor(ClimberConstants.kClimberRetractPercentOutput);
    } else {
      runLeftMotor(0.0);
    }
    if (!rightArmRetracted()) {
      runRightMotor(ClimberConstants.kClimberRetractPercentOutput);
    } else {
      runRightMotor(0.0);
    }
  }

  // public void deployClimber() {
  //   rightClimber.setPositionWithFeedForward(ClimberConstants.kClimberDeployPosition);
  //   leftClimber.setPositionWithFeedForward(ClimberConstants.kClimberDeployPosition);
  // }

  public void deployClimber(){
    if(!leftArmDeployed()){
      runLeftMotor(ClimberConstants.kClimberDeployPercentOutput);
    } else {
      runLeftMotor(0.0);
    }

    if(!rightArmDeployed()){
      runRightMotor(ClimberConstants.kClimberDeployPercentOutput);
    } else {
      runRightMotor(0.0);
    }

  }

  public boolean leftArmRetracted() {
    return leftClimber.getPosition() < ClimberConstants.kClimberRetractPosition;
  }

  public boolean rightArmRetracted() {
    return rightClimber.getPosition() < ClimberConstants.kClimberRetractPosition;
  }

  public boolean leftArmDeployed() {
    return leftClimber.getPosition() > ClimberConstants.kClimberDeployPosition;
  }

  public boolean rightArmDeployed() {
    return rightClimber.getPosition() > ClimberConstants.kClimberDeployPosition;
  }

  public boolean isClimberDeployed(){
    return leftArmDeployed() && rightArmDeployed();
  }

  public void runLeftMotor(double speed) {
    leftClimber.setMotor(speed);
  }

  public void runRightMotor(double speed) {
    rightClimber.setMotor(speed);
  }

  public void stopClimber() {
    leftClimber.setMotor(0);
    rightClimber.setMotor(0);
  }

  public void setBrake() {
    leftClimber.setBrake();
    rightClimber.setBrake();
  }

  public void setCoast() {
    leftClimber.setCoast();
    rightClimber.setCoast();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Left Encoder Reading", leftClimber.getPosition());
    SmartDashboard.putNumber("Climber Right Encoder Reading", rightClimber.getPosition());

    SmartDashboard.putNumber("Left Climber Current", leftClimber.getSupplyCurrent());
    SmartDashboard.putNumber("Right Climber Currrent", rightClimber.getSupplyCurrent());

    SmartDashboard.putBoolean("Climber Deployed", isClimberDeployed());

    // if (SmartDashboard.getBoolean("Climber PID Tuning", false)) {
    // leftClimber.setPIDValues(
    // SmartDashboard.getNumber("Climber P Value", 0),
    // SmartDashboard.getNumber("Climber I Value", 0),
    // SmartDashboard.getNumber("Climber D Value", 0),
    // SmartDashboard.getNumber("Climber FF Value", 0));
    // leftClimber.setPositionWithFeedForward(SmartDashboard.getNumber("Climber
    // Angle Setpoint", 0));
    // }

  }
}
