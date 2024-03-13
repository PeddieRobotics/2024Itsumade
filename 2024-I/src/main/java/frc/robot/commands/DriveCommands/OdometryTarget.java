package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
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

//Turn to target using PID
public class OdometryTarget extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter; 
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput, speakerPoseX, speakerPoseY, targetAngle, currentAngle;
    private PIDController turnPIDController;
    private Logger logger;
    private Pose2d currentOdometry;

    public OdometryTarget() {
        drivetrain = Drivetrain.getInstance();
        logger = Logger.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI, LimelightConstants.kTargetD);
        turnFF = LimelightConstants.kTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;

        currentOdometry = drivetrain.getPose();
        speakerPoseX = LimelightConstants.kSpeakerPositionX;
        speakerPoseY = LimelightConstants.kSpeakerPositionY;

        addRequirements(drivetrain);
        // SmartDashboard.putNumber("Target P", 0);
        // SmartDashboard.putNumber("Target I", 0);
        // SmartDashboard.putNumber("Target D", 0);
        // SmartDashboard.putNumber("Target FF", 0);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Started Odometry Target Command", true);
        limelightShooter.setPipeline(LimelightConstants.kShooterTargetingPipeline);
    }

    @Override
    public void execute() {
        currentOdometry = drivetrain.getPose();
        double deltaX = speakerPoseX - currentOdometry.getX();
        double deltaY = speakerPoseY - currentOdometry.getY();
        targetAngle = Math.atan2(deltaY, deltaX);

        currentAngle = currentOdometry.getRotation().getRadians();

        error = targetAngle - currentAngle;
        
        if (error > turnThreshold) {
            turnInput = turnPIDController.calculate(error) + turnFF;
        } else if (error < -turnThreshold){
            turnInput = turnPIDController.calculate(error) - turnFF;
        } else {
            turnInput = 0;
        }

        drivetrain.drive(oi.getSwerveTranslation(), turnInput * 2, true, oi.getCenterOfRotation());
        // SmartDashboard.putBoolean("Targetting", true);
        // SmartDashboard.putNumber("Target Turn Input", turnInput);
        // SmartDashboard.putBoolean("Limelight has target",
        // limelightShooter.hasTarget());    }
    }

    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putBoolean("Targetting", false);
        drivetrain.stop();
        logger.logEvent("Started Odometry Target Command", false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold && oi.getSwerveTranslation() == new Translation2d(0, 0);
    }
}