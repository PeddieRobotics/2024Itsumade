// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;


public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Arm arm;
  private CANSparkMax armPrimaryMotor, armSecondaryMotor;
  private SparkPIDController armPIDController;

  public Arm() {
    armPrimaryMotor = new CANSparkMax(RobotMap.ARM_PRIMARY_MOTOR, MotorType.kBrushless);
    armSecondaryMotor = new CANSparkMax(RobotMap.ARM_SECONDARY_MOTOR, MotorType.kBrushless);

    armPrimaryMotor.setSmartCurrentLimit(Constants.ARM_PRIMARY_MOTOR_LIMIT);
    armSecondaryMotor.setSmartCurrentLimit(Constants.ARM_SECONDARY_MOTOR_LIMIT);

    armPIDController = armPrimaryMotor.getPIDController();
    armPIDController.setP(Constants.ARM_P);
    armPIDController.setI(Constants.ARM_I);
    armPIDController.setD(Constants.ARM_D);
    armPIDController.setFF(Constants.ARM_FF);
    armPIDController.setIZone(Constants.ARM_IZONE);
  }

  public static Arm getInstance(){
    if (arm == null){
      arm = new Arm();
    }return arm;
  }

  public void updatePID(double P, double I, double D, double FF, double IZONE){
    armPIDController.setP(P);
    armPIDController.setI(I);
    armPIDController.setD(D);
    armPIDController.setFF(FF);
    armPIDController.setIZone(IZONE);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
