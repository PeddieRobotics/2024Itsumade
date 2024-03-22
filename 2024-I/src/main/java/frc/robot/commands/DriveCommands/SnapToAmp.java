package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.LimelightConstants;

public class SnapToAmp extends Command {
    private Drivetrain drivetrain;
    private DriverOI oi;

    private double odometryAngleError, odometryAngleThreshold, odometryTurnFF, turnInput;
    // Odometry Target Values
    private double targetAngle, currentAngle;

    private Pose2d currentOdometry;

    private PIDController odometryTurnPIDController;

    private Logger logger;

    private double initialTime, currentTime;

    public SnapToAmp() {
        drivetrain = Drivetrain.getInstance();
        logger = Logger.getInstance();

        odometryTurnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI, LimelightConstants.kOdometryTargetD);
        odometryTurnPIDController.enableContinuousInput(-180, 180);
        odometryTurnFF = LimelightConstants.kOdometryTargetFF;

        odometryAngleThreshold = LimelightConstants.kAmpAlignAngleThreshold;

        turnInput = 0.0;

        currentOdometry = drivetrain.getPose();
        currentAngle = 0.0;
        odometryAngleError = 0.0;

        targetAngle = LimelightConstants.kAmpOdometryHeading;

        initialTime = 0.0;
        currentTime = 0.0;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Snap To Amp Command", true);

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        currentOdometry = drivetrain.getPose();
        currentAngle = currentOdometry.getRotation().getDegrees();
        odometryAngleError = currentAngle - targetAngle;


        if (odometryAngleError < -odometryAngleThreshold) {
            turnInput = odometryTurnPIDController.calculate(odometryAngleError) + odometryTurnFF;
        } else if (odometryAngleError > odometryAngleThreshold) {
            turnInput = odometryTurnPIDController.calculate(odometryAngleError) - odometryTurnFF;
        } else{
            turnInput = 0;
        }
        drivetrain.drive(oi.getSwerveTranslation(), turnInput, true, oi.getCenterOfRotation());
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Snap To Amp Command", false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(odometryAngleError) < odometryAngleThreshold || (currentTime - initialTime > 1.0);
    }
}