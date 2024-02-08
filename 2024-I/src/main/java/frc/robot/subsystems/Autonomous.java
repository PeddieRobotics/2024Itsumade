package frc.robot.subsystems;

import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase {
  private static SendableChooser<Command> autoChooser;

  private static Autonomous autonomous;
  private Drivetrain drivetrain;
  private Superstructure superstructure;

  public Autonomous() {

    drivetrain = Drivetrain.getInstance();
    superstructure = Superstructure.getInstance();

    registerNamedCommands();
    configureAutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        drivetrain::getPose, // Robot pose supplier
        drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kTranslationP, AutoConstants.kTranslationI, AutoConstants.kTranslationD), // Translation PID constants
            new PIDConstants(AutoConstants.kThetaP, AutoConstants.kThetaI, AutoConstants.kThetaD), // Rotation PID constants
            DriveConstants.kMaxFloorSpeed, // Max module speed, in m/s
            DriveConstants.kBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drivetrain // Reference to this subsystem to set requirements
    );
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> {
      superstructure.requestState(SuperstructureState.GROUND_INTAKE);
    }));
    // NamedCommands.registerCommand("SideLayup", new InstantCommand( () ->
    // {superstructure.shoot(Constants.FlywheelConstants.SideLayupFlywheelSpeed);} )
    // );
    // NamedCommands.registerCommand("FrontLayup", new InstantCommand( () ->
    // {superstructure.shoot(Constants.FlywheelConstants.FrontLayupFlywheelSpeed);}
    // ) );
    NamedCommands.registerCommand("Stow", new InstantCommand(() -> {
      superstructure.requestState(SuperstructureState.STOW);
    }));
  }

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static Autonomous getInstance() {
    if (autonomous == null) {
      autonomous = new Autonomous();
    }
    return autonomous;
  }

  

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //PLACEHOLDER METHOD FOR THE OPERATOR TAB, ONCE THE SKELETON CODE PEOPLE UPDATE MERGE FROM DEV
public Hashtable<String, Command> getAutoRoutines() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutoRoutines'");
}
}
