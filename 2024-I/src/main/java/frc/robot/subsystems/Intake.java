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

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Intake intake;
  public TalonSRX intakeMotor;
  public TalonSRXConfiguration config;

  private DigitalInput intakeSensor;

  public Intake() {
    intakeMotor = new TalonSRX(5);
    SmartDashboard.putNumber("Intake speed", 0);

  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  public void setSpeed(double speed){
    SmartDashboard.putNumber("Intake setpoint", speed);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeMotor.set(TalonSRXControlMode.PercentOutput,SmartDashboard.getNumber("Intake
    // speed", 0));
    SmartDashboard.putNumber("Intake speed", intakeMotor.getMotorOutputPercent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
