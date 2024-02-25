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
    // climberSensor = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_ID);

    rightClimber.setCurrentLimit(ClimberConstants.kClimberCurrentLimit);
    leftClimber.setCurrentLimit(ClimberConstants.kClimberCurrentLimit);

    rightClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);
    leftClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);

    rightClimber.setPIDValues(ClimberConstants.kClimberP,
        ClimberConstants.kClimberI, ClimberConstants.kClimberD,
        ClimberConstants.kClimberFF);
    leftClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD,
        ClimberConstants.kClimberFF);

    rightClimber.setCoast();
    leftClimber.setCoast();

    rightClimber.resetEncoder();
    leftClimber.resetEncoder();

    SmartDashboard.putBoolean("Manual Climber Control", false);
    SmartDashboard.putBoolean("Climber PID Tuning", false);

    SmartDashboard.putNumber("Climber P Value", 0);
    SmartDashboard.putNumber("Climber I Value", 0);
    SmartDashboard.putNumber("Climber D Value", 0);
    SmartDashboard.putNumber("Climber FF Value", 0);

    SmartDashboard.putNumber("Climber Angle Setpoint", 0);

  }

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }

  public boolean climberSensorState() {
    // return climberSensor.get();
    return false;
  }

  public void deployClimber() {
    rightClimber.setPositionWithFeedForward(ClimberConstants.kClimberUnwindPosition);
    leftClimber.setPositionWithFeedForward(ClimberConstants.kClimberUnwindPosition);
  }

  public void retractClimber() {
    if (!isDoneClimbing()) {
      // leftClimber.setMotor(ClimberConstants.kClimberPercentOutput);
    }
  }

  public boolean isDoneClimbing() {
    return climberSensorState();
  }

  public boolean isClimberDeployed() {
    if (leftClimber.getPosition() == ClimberConstants.kClimberUnwindPosition
        && rightClimber.getPosition() == ClimberConstants.kClimberUnwindPosition) {
      return true;
    } else
      return false;
  }

  public void runLeftMotor(double speed) {
    leftClimber.setMotor(speed);
  }

  public void runRigthMotor(double speed) {
    rightClimber.setMotor(speed);
  }

  public void stopClimber() {
    leftClimber.setMotor(0);
    rightClimber.setMotor(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Left Encoder Reading", leftClimber.getPosition());
    SmartDashboard.putNumber("Climber Right Encoder Reading", rightClimber.getPosition());

    SmartDashboard.putNumber("Left Climber Current", leftClimber.getSupplyCurrent());
    SmartDashboard.putNumber("Right Climber Currrent", rightClimber.getSupplyCurrent());

    if (SmartDashboard.getBoolean("Climber PID Tuning", false)) {
      leftClimber.setPIDValues(
          SmartDashboard.getNumber("Climber P Value", 0),
          SmartDashboard.getNumber("Climber I Value", 0),
          SmartDashboard.getNumber("Climber D Value", 0),
          SmartDashboard.getNumber("Climber FF Value", 0));
      leftClimber.setPositionWithFeedForward(SmartDashboard.getNumber("Climber Angle Setpoint", 0));
    }

  }
}
