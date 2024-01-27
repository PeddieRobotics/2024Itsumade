

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.DriverOI;
import frc.robot.utils.OperatorOI;

public class RobotContainer {
  private final Climber climber;
  private final Arm arm;
  private final Autonomous autonomous;
  private final Intake intake;
  private final Flywheel flywheel;
  private final OperatorOI operatorOI;
  private final DriverOI driverOI;

  public Command getAutonomousCommand() {
    return Autonomous.getAutonomousCommand();
  }


  public RobotContainer(){
    arm = Arm.getInstance();
    climber = Climber.getInstance();
    autonomous = Autonomous.getInstance();
    intake = Intake.getInstance();
    flywheel = Flywheel.getInstance();
    operatorOI = OperatorOI.getInstance();
    driverOI = DriverOI.getInstance();
  }
}
