// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class Octopus extends SubsystemBase {

    private static Octopus bubbles;
    private final Kraken kraken;

    public Octopus() {

        kraken = new Kraken(6, "canivore");
        kraken.setCurrentLimit(10);

        SmartDashboard.putBoolean("Run Kraken Percent Output", false);
        SmartDashboard.putNumber("RUN: Kraken Percent Output", 0);
        SmartDashboard.putBoolean("Run Kraken PID", false);

        SmartDashboard.putNumber("Kraken kS Value", 0);
        SmartDashboard.putNumber("Kraken kV Value", 0);
        SmartDashboard.putNumber("Kraken kA Value", 0);
        SmartDashboard.putNumber("Kraken P Value", 0);
        SmartDashboard.putNumber("Kraken I Value", 0);
        SmartDashboard.putNumber("Kraken D Value", 0);
        SmartDashboard.putNumber("Kraken FF Value", 0);
        SmartDashboard.putNumber("Kraken Position Setpoint", 0);
    }

    public static Octopus getInstance(){
        if (bubbles == null){
            bubbles = new Octopus();
        }
        return bubbles;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kraken sMotor Supply Current", kraken.getSupplyCurrent());
        SmartDashboard.putNumber("Kraken Encoder Position", kraken.getPosition());
        SmartDashboard.putNumber("Kraken Encoder Velocity", kraken.getVelocity());
        SmartDashboard.putNumber("Kraken RPM", kraken.getRPM());
        SmartDashboard.putNumber("Kraken Temperature", kraken.getMotorTemperature());        

        if(SmartDashboard.getBoolean("Run Kraken Percent Output", false)){
            kraken.setMotor(SmartDashboard.getNumber("RUN: Kraken Percent Output", 0));
        } else if (SmartDashboard.getBoolean("Run Kraken PID", false)){
            kraken.setVelocityPIDValues(
                SmartDashboard.getNumber("Kraken kS Value", 0),
                SmartDashboard.getNumber("Kraken kV Value", 0),
                SmartDashboard.getNumber("Kraken kA Value", 0),         
                SmartDashboard.getNumber("Kraken P Value", 0),
                SmartDashboard.getNumber("Kraken I Value", 0),
                SmartDashboard.getNumber("Kraken D Value", 0),
                SmartDashboard.getNumber("Kraken FF Value", 0));
            kraken.setPositionWithFeedForward(SmartDashboard.getNumber("Kraken Position Setpoint", 0));
        }
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}