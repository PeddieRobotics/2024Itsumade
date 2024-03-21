// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Lights lights;
  private final CANdle candle;
  private boolean isClimbing;
  private double lastIntook;

  public enum LightState {
    IDLE,
    INTAKING,
    INTOOK,
    HAS_TARGET,
    CLIMBING,
    TARGETED,
    DONE_CLIMBING,
    FAILED
  }

  LightState systemState;
  LightState requestedSystemState;

  public Lights() {
    systemState = LightState.IDLE;
    requestedSystemState = LightState.IDLE;

    candle = new CANdle(0);

    SmartDashboard.putString("LED state", systemState.toString());
  }

  public static Lights getInstance() {
    if (lights == null) {
      lights = new Lights();
    }
    return lights;

  }

  public void requestState(LightState request) {
    if(request == LightState.INTOOK){
      lastIntook = Timer.getFPGATimestamp();
    }

    //reset animation so easy to change
    candle.clearAnimation(0);
    candle.setLEDs(0,0,0);
    requestedSystemState = request;
  }

  public LightState getLightState(){
    return systemState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED state", systemState.toString());
    switch (systemState) {
      case IDLE:
        candle.setLEDs(0,0,0);
        break;

      case INTAKING:
        //candle.animate(new StrobeAnimation(255, 0, 0,0,1,8), 0);
        candle.setLEDs(255,0,0);
        break;

      case INTOOK:
        candle.animate(new StrobeAnimation(0, 153, 0,0,0.3,8), 0);
        if(Timer.getFPGATimestamp() - lastIntook > 1.5){
          requestState(LightState.IDLE);
        }
        break;

      case HAS_TARGET:
        candle.setLEDs(0,0,255);
        break;

      case CLIMBING:
        candle.animate(new ColorFlowAnimation(255, 255, 0, 0, 0.3, 8, Direction.Forward), 0);
        break;
      
      case DONE_CLIMBING:
        candle.animate(new RainbowAnimation(), 0);
        break;

      case TARGETED:
        candle.setLEDs(0,255,0);
        break;

      case FAILED:
        candle.setLEDs(255,0,0);
        break;
    }

    systemState = requestedSystemState;

  }

  public String stateAsString(){
    switch (systemState) {
      case IDLE:
        return "IDLE";
      case INTAKING:
        return "INTAKING";
      case INTOOK: 
        return "INTOOK";
      case HAS_TARGET:
        return "HAS_TARGET";
      case CLIMBING:
        return "CLIMBING";
      case TARGETED:
        return "TARGETED";
      case DONE_CLIMBING:
        return "DONE_CLIMBING";
      case FAILED:
        return "FAILED";
    }
    return "";
  }

  public String getLightStateAsString(){
    return systemState.toString();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
