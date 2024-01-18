// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private final Climber climber; 
  private final Arm arm; 
  private final Autonomous autonomous;

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autonomous.getAutonomousCommand();
  }

  public RobotContainer(){
    arm = Arm.getInstance();
    climber = Climber.getInstance();
    autonomous = Autonomous.getInstance();
  }
}
