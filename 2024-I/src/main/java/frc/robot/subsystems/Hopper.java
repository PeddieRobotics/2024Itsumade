// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.HopperConstants;
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

  public void runHopper(){
    //indexing (but not shooting logic) here
    setHopper(HopperConstants.kHopperSpeed);
  }

  public void runHopperHP(){
    setHopper(-HopperConstants.kHopperSpeed);
  }

  public void feed(){
    setHopper(HopperConstants.kFeedSpeed);
  }

  public void setHopper(double speed) {
    hopperMotor.setMotor(speed);
  }

  public void stopHopper() {
    hopperMotor.setMotor(0);
  }

  //returns if the beam is broken
  public boolean getTopSensor() {
    return !topHopperSensor.get();
  }

  //returns if the beam is broken
  public boolean getBottomSensor() {
    return !bottomHopperSensor.get();
  }

  public boolean hasGamepiece(){
    return (getTopSensor() || getBottomSensor());
  }

  public boolean isGamepieceIndexed(){
    return (getTopSensor() && getBottomSensor());
  }

  public double getMotorCurrent(){
    return hopperMotor.getSupplyCurrent();
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
