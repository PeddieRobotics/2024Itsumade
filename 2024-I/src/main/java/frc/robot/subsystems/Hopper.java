// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Hopper hopper;
  private Kraken hopperMotor;

  // public LaserCan topHopperSensor, bottomHopperSensor;
  private DigitalInput topHopperSensor, bottomHopperSensor;

  public Hopper() {
    hopperMotor = new Kraken(RobotMap.HOPPER_MOTOR_CAN_ID, RobotMap.CANIVORE_NAME);
    topHopperSensor = new DigitalInput(RobotMap.TOP_HOPPER_SENSOR_ID);
    bottomHopperSensor = new DigitalInput(RobotMap.BOTTOM_HOPPER_SENSOR_ID);

    hopperMotor.setCurrentLimit(IntakeConstants.kHopperCurrentLimit);
    hopperMotor.setBrake();
  }

  public static Hopper getInstance() {
    if (hopper == null) {
      hopper = new Hopper();
    }
    return hopper;
  }

  public void index(){
    //indexing (but not shooting logic) here
    setHopper(Constants.HopperConstants.kFloorIndexSpeed);
  }

  public void hpIndex(){
    setHopper(Constants.HopperConstants.kHPIndexSpeed);
  }

  public void feed(){
    setHopper(Constants.HopperConstants.kFeedSpeed);
  }

  public void setHopper(double speed) {
    hopperMotor.setMotor(speed);
  }

  public void stopHopper() {
    hopperMotor.setMotor(0);
  }

  //returns is the beam is broken
  public boolean topSensor() {
    return !topHopperSensor.get();
  }

  //returns is the beam is broken
  public boolean bottomSensor() {
    return !bottomHopperSensor.get();
  }

  // public double getTopSensorMeasurement() {
  //   return topHopperSensor.getMeasurement().distance_mm;
  // }

  // public double getBottomSensorMeasurement() {
  //   return bottomHopperSensor.getMeasurement().distance_mm;
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
