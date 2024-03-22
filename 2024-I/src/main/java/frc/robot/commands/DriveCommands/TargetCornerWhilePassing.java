package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.LimelightConstants;

//Turn to target using PID
public class TargetCornerWhilePassing extends Command {
    private Drivetrain drivetrain;
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput, speakerPoseX, speakerPoseY, targetAngle, currentAngle;
    private PIDController turnPIDController;
    private Logger logger;
    private Pose2d currentOdometry;
    private Lights lights;

    public TargetCornerWhilePassing() {
        drivetrain = Drivetrain.getInstance();
        logger = Logger.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI, LimelightConstants.kOdometryTargetD);
        turnPIDController.enableContinuousInput(-180, 180);
        turnFF = LimelightConstants.kOdometryTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;

        currentOdometry = drivetrain.getPose();
        
        speakerPoseX = 0;
        speakerPoseY = 0;

        lights = Lights.getInstance();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Target Corner While Passing Command", true);
        
        if(DriverStation.getAlliance().get() == Alliance.Red){
            speakerPoseX = LimelightConstants.kRedCornerPassingX;
            speakerPoseY = LimelightConstants.kRedCornerPassingY;
        }
        else{
            speakerPoseX = LimelightConstants.kBlueCornerPassingX;
            speakerPoseY = LimelightConstants.kBlueCornerPassingY;
        }

    }

    @Override
    public void execute() {
        currentOdometry = drivetrain.getPose();
        double deltaX = speakerPoseX - currentOdometry.getX();
        double deltaY = speakerPoseY - currentOdometry.getY();
        targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        currentAngle = currentOdometry.getRotation().getDegrees();

        error = currentAngle - targetAngle;

        if (Math.abs(error) >= 3 * turnThreshold)
            lights.requestState(LightState.HAS_TARGET);
        else
            lights.requestState(LightState.TARGETED);
        
        if (error > turnThreshold) {
            turnInput = turnPIDController.calculate(error) + turnFF;
        } else if (error < -turnThreshold){
            turnInput = turnPIDController.calculate(error) - turnFF;
        } else {
            turnInput = 0;
        }

        drivetrain.drive(oi.getSwerveTranslation(), turnInput * 2, true, oi.getCenterOfRotation());

    }

    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putBoolean("Targetting", false);
        drivetrain.stop();
        logger.logEvent("Target Corner While Passing Command", false);
        lights.requestState(LightState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}