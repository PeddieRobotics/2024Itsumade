package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

// Turns to target by precalculating a gyro angle to goal based on Limelight TX
public class SnapToSpeaker extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput;
    private PIDController turnPIDController;
    private Logger logger;
    private double pastHeading, currentHeading;

    private double latency;
    private double setpoint;
    private double initialTime, currentTime;

    public SnapToSpeaker() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kSnapToSpeakerP, LimelightConstants.kSnapToSpeakerI,
                LimelightConstants.kSnapToSpeakerD);
        turnPIDController.setIZone(4.0);

        turnFF = LimelightConstants.kSnapToSpeakerFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;
        logger = Logger.getInstance();

        pastHeading = drivetrain.getHeading();
        currentHeading = drivetrain.getHeading();
        setpoint = 0.0;

        latency = 0.0;

        initialTime = 0.0;
        currentTime = 0.0;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Snap to Speaker Command", true);
        limelightShooter.setPipeline(LimelightConstants.kShooterPipeline);

        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
        } else {
            LimelightShooter.getInstance().setPriorityTag(7);
        }

        latency = limelightShooter.getTotalLatencyInMS();
        pastHeading = drivetrain.getPastHeading(latency);
        setpoint = pastHeading + limelightShooter.getTxAverage();
        turnPIDController.setSetpoint(setpoint);

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if (limelightShooter.hasTarget()) {
            currentHeading = drivetrain.getHeading();
            error = currentHeading - setpoint;

            if (error < -turnThreshold) {
                turnInput = turnPIDController.calculate(currentHeading) + turnFF;
            } else if (error > turnThreshold) {
                turnInput = turnPIDController.calculate(currentHeading) - turnFF;
            } else {
                turnInput = 0;
            }
        } else {
            turnInput = 0;
        }

        drivetrain.drive(oi.getSwerveTranslation(), -turnInput, true, oi.getCenterOfRotation());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Snap to Speaker Command", false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold || (currentTime - initialTime > 1.0);
    }
}