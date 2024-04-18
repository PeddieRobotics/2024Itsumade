package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

//Turn to target using PID
public class TargetInAuto extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private Lights lights;
    private DriverOI oi;

    private double speakerPoseX, speakerPoseY, odometryTurnFF;
    private double error, turnThreshold, turnFF, turnInput, initialTime, currentTime, target;
    private PIDController turnPIDController, odometryTurnPIDController;
    private Logger logger;

    public TargetInAuto() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        lights = Lights.getInstance();
        logger = Logger.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetAutoP, LimelightConstants.kTargetAutoI,
                LimelightConstants.kTargetAutoD);
        turnPIDController.setIZone(LimelightConstants.kTargetAutoIZone);

        odometryTurnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI,
                LimelightConstants.kOdometryTargetD);
        odometryTurnPIDController.enableContinuousInput(-180, 180);

        turnFF = LimelightConstants.kTargetAutoFF;
        odometryTurnFF = LimelightConstants.kOdometryTargetFF;
        turnThreshold = LimelightConstants.kTargetAutoThreshold;
        turnInput = 0;
        target = LimelightConstants.kRedTargetTarget;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            target = LimelightConstants.kBlueTargetTarget;

        initialTime = 0.0;
        currentTime = 0.0;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Target Command", true);
        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        if(DriverStation.getAlliance().get() == Alliance.Red){
            target = limelightShooter.getRedTargetingOffset();
            LimelightShooter.getInstance().setPriorityTag(4);
            speakerPoseX = LimelightConstants.kRedSpeakerPositionX;
            speakerPoseY = LimelightConstants.kRedSpeakerPositionY;
        } else {
            target = limelightShooter.getBlueTargetingOffset();
            LimelightShooter.getInstance().setPriorityTag(7);
            speakerPoseX = LimelightConstants.kBlueSpeakerPositionX;
            speakerPoseY = LimelightConstants.kBlueSpeakerPositionY;
        }
    }

    @Override
    public void execute() {
        if(limelightShooter.hasTarget()){
            if (Math.abs(error) >= 3 * turnThreshold)
                lights.requestState(LightState.HAS_TARGET);
            else
                lights.requestState(LightState.TARGETED);
        } 

        currentTime = Timer.getFPGATimestamp();
        if (limelightShooter.hasTarget()) {
            error = limelightShooter.getTxAverage() - target;
            if (error < -turnThreshold) {
                turnInput = turnPIDController.calculate(error) + turnFF;
            } else if (error > turnThreshold) {
                turnInput = turnPIDController.calculate(error) - turnFF;
            } else {
                turnInput = 0;
            }
            // it was turnInput * 2 before in this command
            // but odometry was not in HybridTarget
            // so we multiply here to keep things as before
            turnInput *= 2;
        } else {
            var currentOdometry = drivetrain.getPose();
            double deltaX = speakerPoseX - currentOdometry.getX();
            double deltaY = speakerPoseY - currentOdometry.getY();
            double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

            double currentAngle = currentOdometry.getRotation().getDegrees();

            error = currentAngle - targetAngle - target;

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
        logger.logEvent("Target Command, offset " + target, false);
        lights.requestState(LightState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold || (currentTime - initialTime > 1.0);
    }
}