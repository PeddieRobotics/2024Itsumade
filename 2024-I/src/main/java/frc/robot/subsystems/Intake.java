// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Intake intake;
  public TalonSRX intakeMotor;
  public TalonSRXConfiguration config;

  //public LaserCan intakeSensor;

  public Intake() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR_CAN_ID);
   // intakeSensor = new LaserCan(RobotMap.INTAKE_SENSOR_ID);

    config = new TalonSRXConfiguration();
    config.peakCurrentLimit = IntakeConstants.kIntakeCurrentLimit;
    intakeMotor.configAllSettings(config);

    intakeMotor.enableCurrentLimit(true);
  }

  public static Intake getInstance(){
    if(intake == null){
      intake = new Intake();
    }return intake;
  }

  public void setIntake(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopIntake(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public boolean getSensorReading(){
    if(getSensorMeasurement() < IntakeConstants.kIntakeSensorThreshold){
      return true;
    }
    return false;
  }

  public double getSensorMeasurement(){
    //returns sensor distance in mm
    //return intakeSensor.getMeasurement().distance_mm;
    return 0;
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
