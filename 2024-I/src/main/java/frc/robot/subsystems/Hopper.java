// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //TODO: figure out if sensor will be digital input or analog
  private AnalogInput topHopperSensor, bottomHopperSensor;

  public Hopper() {
    hopperMotor = new Kraken(RobotMap.HOPPER_MOTOR_CAN_ID, RobotMap.CANIVORE_NAME);

    topHopperSensor = new AnalogInput(RobotMap.TOP_HOPPER_SENSOR_ID);
    bottomHopperSensor = new AnalogInput(RobotMap.BOTTOM_HOPPER_SENSOR_ID);

    hopperMotor.setSupplyCurrentLimit(IntakeConstants.kHopperCurrentLimit);
    hopperMotor.setBrake();

    SmartDashboard.putBoolean("Hopper Percent Output", false);
    SmartDashboard.putNumber("Hopper Motor Percent Output", 0);
  }

  public static Hopper getInstance() {
    if (hopper == null) {
      hopper = new Hopper();
    }
    return hopper;
  }

  public void runHopperGroundIntake(){
    //indexing (but not shooting logic) here
    setHopper(HopperConstants.kGroundIntakeHopperSpeed);
  }

  public void runHopperHPIntake(){
    setHopper(-HopperConstants.kHPIntakeHopperSpeed);
  }

  public void feedFlywheelLayup(){
    setHopper(HopperConstants.kFeedFlywheelLayupSpeed);
  }

  public void feedFlywheelAmp() {
    setHopper(HopperConstants.kFeedFlywheelAmpSpeed);
  }

    public void feedFlywheelSpeaker() {
    setHopper(HopperConstants.kFeedFlywheelSpeakerSpeed);
  }

  public void setHopper(double speed) {
    hopperMotor.setMotor(speed);
  }

  public void stopHopper() {
    hopperMotor.setMotor(0);
  }

  public boolean getTopSensor(){
    return getTopSensorReading() < 0.03;
  }

  public double getTopSensorReading() {
    return topHopperSensor.getVoltage();
  }

  public boolean getBottomSensor(){
    return getBottomSensorReading() < 0.03;
  }

  public double getBottomSensorReading() {
    return bottomHopperSensor.getVoltage();
  }

  public boolean hasGamepiece(){
    return (getTopSensor() || getBottomSensor());
  }

  public boolean isGamepieceIndexed(){
    return getTopSensor();
    // return (getTopSensor() && getBottomSensor());
  }

  public double getMotorCurrent(){
    return hopperMotor.getSupplyCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Hopper full", isGamepieceIndexed());
    SmartDashboard.putBoolean("Hopper Top Sensor Status", getTopSensor());
    SmartDashboard.putBoolean("Hopper Bottom Sensor Status", getBottomSensor());
    // SmartDashboard.putNumber("Hopper Top Sensor Reading", getTopSensorReading());
    // SmartDashboard.putNumber("Hopper Bottom Sensor Reading", getBottomSensorReading());
    // if (SmartDashboard.getBoolean("Hopper Percent Output", false)){
    //   hopperMotor.setMotor(SmartDashboard.getNumber("Hopper Motor Percent Output", 0));
    // }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
