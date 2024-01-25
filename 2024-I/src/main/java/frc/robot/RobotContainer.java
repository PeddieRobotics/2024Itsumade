// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Octopus;


public class RobotContainer {
  // private final Climber climber; 
  // private final Arm arm; 
  // private final Octopus bubbles;
  private final Drivetrain drivetrain;
  // private final Autonomous autonomous;

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autonomous.getAutonomousCommand();
    return null;
  }

  public RobotContainer(){
    // arm = Arm.getInstance();
    // climber = Climber.getInstance();
    drivetrain = Drivetrain.getInstance();
    // bubbles = Octopus.getInstance();
    // autonomous = Autonomous.getInstance();

    drivetrain.setDefaultCommand(new SwerveDriveCommand());
  }
}
