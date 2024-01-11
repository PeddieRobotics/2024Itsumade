

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;

public class RobotContainer {
  public final Climber climber;
  private final Arm arm;
  private final Autonomous autonomous;

  public Command getAutonomousCommand() {
    return Autonomous.getAutonomousCommand();
  }


  public RobotContainer(){
    arm = Arm.getInstance();
    climber = Climber.getInstance();
    autonomous = Autonomous.getInstance();
  }
}
