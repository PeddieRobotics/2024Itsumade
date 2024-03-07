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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.FollowNoteInAuto;
import frc.robot.commands.DriveCommands.PathPlannerToPoint;
import frc.robot.commands.DriveCommands.PathPlannerToShoot;
import frc.robot.commands.DriveCommands.Target;
import frc.robot.commands.DriveCommands.TargetInAuto;
// import frc.robot.commands.DriveCommands.ForcedCalibration;
// import frc.robot.commands.DriveCommands.TurnOffMegatag;
// import frc.robot.commands.DriveCommands.TurnOnMegatag;
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
    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
                drivetrain::getPose, // Robot pose supplier
                drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(AutoConstants.kTranslationP, AutoConstants.kTranslationI,
                                AutoConstants.kTranslationD), // Translation PID constants
                        new PIDConstants(AutoConstants.kThetaP, AutoConstants.kThetaI, AutoConstants.kThetaD), // Rotation
                                                                                                               // PID
                                                                                                               // constants
                        DriveConstants.kMaxFloorSpeed, // Max module speed, in m/s
                        DriveConstants.kBaseRadius, // Drive base radius in meters. Distance from robot center to
                                                    // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE RED SIDE (FOR THE LAB)

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return true;
                },
                drivetrain // Reference to this subsystem to set requirements
        );
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Stow", new InstantCommand(
            () -> superstructure.requestState(SuperstructureState.STOW)
        ));
        NamedCommands.registerCommand("Intake", new InstantCommand(
            () -> superstructure.requestState(SuperstructureState.GROUND_INTAKE)
        ));
        NamedCommands.registerCommand("LL Prep", new ParallelDeadlineGroup(
            new WaitCommand(AutoConstants.kLimelightPrepDeadlineTime),
            new InstantCommand(() -> superstructure.requestState(SuperstructureState.LL_PREP))
        ));
        NamedCommands.registerCommand("Layup Prep", new ParallelDeadlineGroup( //refactor to 'front layup prep' after hatboro
            new WaitCommand(AutoConstants.kLayupPrepDeadlineTime),
            new InstantCommand(() -> superstructure.requestState(SuperstructureState.FRONT_LAYUP_PREP))
        ));
        NamedCommands.registerCommand("Side Layup Prep", new ParallelDeadlineGroup(
            new WaitCommand(AutoConstants.kLayupPrepDeadlineTime),
            new InstantCommand(() -> superstructure.requestState(SuperstructureState.SIDE_LAYUP_PREP))
        ));
        NamedCommands.registerCommand("O Path Shot Prep", new ParallelDeadlineGroup(
            new WaitCommand(AutoConstants.kLayupPrepDeadlineTime),
            new InstantCommand(() -> superstructure.requestState(SuperstructureState.CUSTOM_SHOT_PREP,52)) //tune val i guess
        ));
        NamedCommands.registerCommand("Score", new ParallelDeadlineGroup(
            new WaitCommand(AutoConstants.kScoreDeadlineTime),
            new InstantCommand(() -> superstructure.sendToScore())
        ));
        NamedCommands.registerCommand("Target", new TargetInAuto());

        // NamedCommands.registerCommand("Set Odom", new ForcedCalibration());
        // NamedCommands.registerCommand("Turn on MegaTag", new TurnOnMegatag());
        // NamedCommands.registerCommand("Turn off MegaTag", new TurnOffMegatag());

        NamedCommands.registerCommand("W ToClosestShooting", new PathPlannerToShoot(4));
        NamedCommands.registerCommand("X SeekNote", new FollowNoteInAuto(1));
        NamedCommands.registerCommand("Y ToTopSeekNoteLocation", new PathPlannerToPoint(6.36, 6.65, 0, 4));
        NamedCommands.registerCommand("Z ToBottomSeekNoteLocation", new PathPlannerToPoint(6.36, 1.64, 0, 4));
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public static Autonomous getInstance() {
        if (autonomous == null) {
            autonomous = new Autonomous();
        }
        return autonomous;
    }
}
