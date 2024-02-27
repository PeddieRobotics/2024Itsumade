
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.Shuffleboard.ShuffleboardMain;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.OperatorOI;

public class RobotContainer {
  private final Arm arm;
  private final Climber climber;
  private final Autonomous autonomous;
  private final Drivetrain drivetrain;
  private final Hopper hopper;
  private final Intake intake;
  private final LimelightShooter limelightShooter;
  private final Flywheel flywheel;
  private final Superstructure superstructure;
  private ShuffleboardMain shuffleboard;
  private final OperatorOI operatorOI;
  private final DriverOI driverOI;

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autonomous.getAutonomousCommand();
  }

  public RobotContainer() {

    arm = Arm.getInstance();
    climber = Climber.getInstance();
    drivetrain = Drivetrain.getInstance();
    hopper = Hopper.getInstance();
    autonomous = Autonomous.getInstance();
    intake = Intake.getInstance();
    limelightShooter = LimelightShooter.getInstance();
    flywheel = Flywheel.getInstance();
    superstructure = Superstructure.getInstance();
    shuffleboard = ShuffleboardMain.getInstance();
    operatorOI = OperatorOI.getInstance();
    driverOI = DriverOI.getInstance();
    // shuffleboardMain = ShuffleboardMain.getInstance();

    drivetrain.setDefaultCommand(new SwerveDriveCommand());
    limelightShooter.setPipeline(Constants.LimelightConstants.kShooterAprilTagPipeline);
  }

  public void resetGyro() {
    drivetrain.resetGyro();
  }
}
