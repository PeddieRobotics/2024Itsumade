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

    rightClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD, ClimberConstants.kClimberFF);
    leftClimber.setPIDValues(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD, ClimberConstants.kClimberFF);

    rightClimber.setBrake();
    leftClimber.setBrake();
    
    SmartDashboard.putNumber("Manual Climber Speed", 0);
    SmartDashboard.putBoolean("Manual Climber Control", false);
  }

  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
    } return climber;
  }

  public boolean climberSensorState() {
    // return climberSensor.get();
    return false;
  }

  public void deployClimber(){
    // leftClimber.setPosition(ClimberConstants.kClimberUnwindPosition);
  }

  public void pulldownClimber(){
    if(!isDoneClimbing()){
      // leftClimber.setMotor(ClimberConstants.kClimberPercentOutput);
    }
  }

  public boolean isDoneClimbing(){
    return climberSensorState();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(SmartDashboard.getBoolean("Manual Climber Control", true)){
      leftClimber.setMotor(DriverOI.getInstance().getForward());
      rightClimber.setMotor(DriverOI.getInstance().getForward());
    } 

  }
}
