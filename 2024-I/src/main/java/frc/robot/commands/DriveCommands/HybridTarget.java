package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.LimelightConstants;

public class HybridTarget extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;
    private Lights lights;

    private double error, turnThreshold, llTurnFF, odometryTurnFF, turnInput;
    // Odometry Target Values
    private double speakerPoseX, speakerPoseY, targetAngle, currentAngle;
    private Pose2d currentOdometry;

    private PIDController llTurnPIDController, odometryTurnPIDController;
    private Logger logger;

    public HybridTarget() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        logger = Logger.getInstance();
        lights = Lights.getInstance();

        llTurnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
                LimelightConstants.kTargetD);
        odometryTurnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI, LimelightConstants.kOdometryTargetD);
        odometryTurnPIDController.enableContinuousInput(-180, 180);
        llTurnFF = LimelightConstants.kTargetFF;
        odometryTurnFF = LimelightConstants.kOdometryTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;
        currentOdometry = drivetrain.getPose();

        speakerPoseX = 0;
        speakerPoseY = 0;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Hybrid Target Command", true);

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            speakerPoseX = LimelightConstants.kRedSpeakerPositionX;
            speakerPoseY = LimelightConstants.kRedSpeakerPositionY;
        } else {
            speakerPoseX = LimelightConstants.kBlueSpeakerPositionX;
            speakerPoseY = LimelightConstants.kBlueSpeakerPositionY;
        }

        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
        } else {
            LimelightShooter.getInstance().setPriorityTag(7);
        }
    }

    @Override
    public void execute() {

        if(!limelightShooter.hasTarget()){
            lights.requestState(LightState.FAILED);
        } else if(limelightShooter.hasTarget() && lights.getLightState() != LightState.TARGETED){
            lights.requestState(LightState.HAS_TARGET);
        } 
        
        if (limelightShooter.hasTarget() && Math.abs(limelightShooter.getTxAverage()) < 20.0) {

            error = limelightShooter.getTxAverage();
            if (error < -turnThreshold) {
                turnInput = llTurnPIDController.calculate(error) + llTurnFF;
            } else if (error > turnThreshold) {
                turnInput = llTurnPIDController.calculate(error) - llTurnFF;
            } else {
                turnInput = 0;
            }
        } else {
            currentOdometry = drivetrain.getPose();
            double deltaX = speakerPoseX - currentOdometry.getX();
            double deltaY = speakerPoseY - currentOdometry.getY();
            targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

            currentAngle = currentOdometry.getRotation().getDegrees();

            error = currentAngle - targetAngle;

            if (error > turnThreshold) {
                turnInput = odometryTurnPIDController.calculate(error) + odometryTurnFF;
            } else if (error < -turnThreshold) {
                turnInput = odometryTurnPIDController.calculate(error) - odometryTurnFF;
            } else {
                turnInput = 0;
            }
        }
        drivetrain.drive(oi.getSwerveTranslation(), turnInput, true, oi.getCenterOfRotation());

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Hybrid Target Command", false);
        
        if(Math.abs(error) < turnThreshold && limelightShooter.hasTarget()){
            lights.requestState(LightState.TARGETED);
        } else {
            lights.requestState(LightState.IDLE);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold && oi.getSwerveTranslation() == new Translation2d(0, 0);
    }
}