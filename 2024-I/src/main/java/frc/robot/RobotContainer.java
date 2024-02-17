

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.DriverOI;

public class RobotContainer {
  // private final Climber climber;
  private final Arm arm;
  private final Autonomous autonomous;
  private final Drivetrain drivetrain;
  private final Intake intake;
  // private final Flywheel flywheel;
  // private final OperatorOI operatorOI;
  private final DriverOI driverOI;
  private final LimelightShooter limelightShooter;

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autonomous.getAutonomousCommand();
  }

  public RobotContainer(){
    
    arm = Arm.getInstance();
    // climber = Climber.getInstance();
    drivetrain = Drivetrain.getInstance();
    autonomous = Autonomous.getInstance();
    intake = Intake.getInstance();
    limelightShooter = LimelightShooter.getInstance();
    // flywheel = Flywheel.getInstance();
    // operatorOI = OperatorOI.getInstance();
    driverOI = DriverOI.getInstance();
    //shuffleboardMain = ShuffleboardMain.getInstance();

    // drivetrain.setDefaultCommand(new SwerveDriveCommand());
  }

  public void resetGyro(){
    drivetrain.resetGyro();
  }
  // public void runControlLoop(){
  //   driverOI.controlLoop();
  // }
}
