// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CTREConfigs;

public class SwerveModule extends SubsystemBase {

  private final TalonFX drivingMotor;
  private final TalonFX steeringMotor;
  private final CANcoder steerEncoder;

  private final int drivingCANId;
  private final int steeringCANId;
  private final int CANCoderId;
  
  private SwerveModuleState state;
  private SwerveModuleState desired_state = new SwerveModuleState(0.0, new Rotation2d(0));

  public SwerveModule(int drivingCANId, int steeringCANId, int CANCoderId) {
    this.drivingCANId = drivingCANId;
    this.steeringCANId = steeringCANId;
    this.CANCoderId = CANCoderId;

    // Setup Driving Motor
    drivingMotor = new TalonFX(drivingCANId);
    drivingConfig






    steeringMotor = new TalonFX(steeringCANId);

    steerEncoder = new CANcoder(CANCoderId);
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
