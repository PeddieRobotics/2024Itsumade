// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
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
    climberSensor = new DigitalInput(ClimberConstants.CLIMBER_SENSOR_ID);

    rightClimber.setCurrentLimit(ClimberConstants.kClimberRightCurrentLimit);
    leftClimber.setCurrentLimit(ClimberConstants.kClimberLeftCurrentLimit);

    rightClimber.setFollower(RobotMap.CLIMBER_LEFT_MOTOR, false);

    rightClimber.setFeedbackDevice(0, FeedbackSensorSourceValue.RemoteCANcoder);
    leftClimber.setFeedbackDevice(RobotMap.CLIMBER_CANCODER_ID, FeedbackSensorSourceValue.RemoteCANcoder);

    rightClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);
    leftClimber.setVelocityConversionFactor(ClimberConstants.kClimberGearReduction);

    rightClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD, ClimberConstants.kClimberFF);
    leftClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD, ClimberConstants.kClimberFF);

    rightClimber.setBrake();
    leftClimber.setBrake();
  }

  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
    } return climber;
  }

  public boolean climberSensorState() {
    return climberSensor.get();
  }

  public void deployClimber(){
    rightClimber.setPosition(ClimberConstants.kClimberUnwindPosition);
  }

  public void pulldownClimber(){
    rightClimber.setMotor(ClimberConstants.kClimberPercentOutput);
  }

  public boolean doneClimbing(){
    return climberSensorState();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
