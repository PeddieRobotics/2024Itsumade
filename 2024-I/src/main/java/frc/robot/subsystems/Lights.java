// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;

public class Lights extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public static Lights lights;
  private final CANdle candle;
  private boolean isClimbing;

  public enum LightState {
    IDLE,
    INTAKING,
    INTOOK,
    HAS_TARGET,
    CLIMBING,
    TARGETED,

  }

  LightState systemState;
  LightState nextSystemState;
  LightState requestedSystemState;

  public Lights() {
    systemState = LightState.IDLE;
    nextSystemState = LightState.IDLE;
    requestedSystemState = LightState.IDLE;

    candle = new CANdle(0);
  }

  public static Lights getInstance() {
    if (lights == null) {
      lights = new Lights();
    }
    return lights;

  }

  public void requestState(LightState request) {
    requestedSystemState = request;
  }

  @Override
  public void periodic() {
    switch (systemState) {
      case IDLE:
        candle.clearAnimation(0);
        break;

      case INTAKING:
        candle.animate(new StrobeAnimation(255, 0, 0), 0);
        break;

      case INTOOK:
        candle.animate(new StrobeAnimation(0, 153, 0), 0);
        break;

      case HAS_TARGET:
        candle.animate(new StrobeAnimation(0, 0, 255), 0);
        break;

      case CLIMBING:
        candle.animate(new RainbowAnimation(), 0);
        break;

      case TARGETED:
        candle.animate(new LarsonAnimation(255, 0, 127), 0);
        break;

      
    }

    systemState = requestedSystemState;

  }

  public void setIsClimbing(boolean climbing){
    isClimbing = climbing;
  }

  public boolean getIsClimbing(){
    return isClimbing;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
