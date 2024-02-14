// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Intake intake;
  public TalonSRX intakeMotor;
  public TalonSRXConfiguration config;

  private AnalogInput intakeSensor;

  public Intake() {
    intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR_CAN_ID);
    intakeSensor = new AnalogInput(RobotMap.INTAKE_SENSOR_ID);
    SmartDashboard.putNumber("Intake speed", 0);

    config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = IntakeConstants.kIntakeCurrentLimit;
    intakeMotor.configAllSettings(config);

    intakeMotor.enableCurrentLimit(true);
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  public void setIntake(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  // returns if beam is broken
  public boolean getSensor(){
    return getSensorReading() < 0.01;
  }

  public double getSensorReading() {
    return intakeSensor.getVoltage();
  }

  public double getMotorCurrent() {
    return intakeMotor.getSupplyCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeMotor.set(TalonSRXControlMode.PercentOutput,SmartDashboard.getNumber("Intake
    // speed", 0));
    SmartDashboard.putNumber("Intake Motor Current", getMotorCurrent());
    SmartDashboard.putNumber("Intake Sensor Reading", getSensorReading());
    SmartDashboard.putBoolean("Intake Sensor Status", getSensor());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
