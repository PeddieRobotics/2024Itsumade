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
    private final SwerveModule module;

    public Octopus() {

       module = new SwerveModule("canivore", 5, 6, 1);
       module.resetCANCoder();
       module.resetDriveEncoder();
       module.resetTurnEncoder();
    }

    public static Octopus getInstance(){
        if (bubbles == null){
            bubbles = new Octopus();
        }
        return bubbles;
    }

    @Override
    public void periodic() {
       
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}