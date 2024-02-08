

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.DriverOI;
import frc.robot.utils.OperatorOI;

public class RobotContainer {
  // private final Climber climber;
  // private final Arm arm;
  // private final Autonomous autonomous;
  //private final Drivetrain drivetrain;
  //private final Intake intake;
  //private final ShuffleboardMain shuffleboardMain;
  // private final Flywheel flywheel;
  // private final OperatorOI operatorOI;
  private final DriverOI driverOI;

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autonomous.getAutonomousCommand();
    return null;
  }

  public RobotContainer(){
    
    // arm = Arm.getInstance();
    // climber = Climber.getInstance();
    // autonomous = Autonomous.getInstance();
    //drivetrain = Drivetrain.getInstance();
    //intake = Intake.getInstance();
    // flywheel = Flywheel.getInstance();
    // operatorOI = OperatorOI.getInstance();
    driverOI = DriverOI.getInstance();
    //shuffleboardMain = ShuffleboardMain.getInstance();

    //drivetrain.setDefaultCommand(new SwerveDriveCommand());
  }

  public void runControlLoop(){
    driverOI.controlLoop();
  }
}
