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
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.LimelightConstants;

// Turns to target by precalculating a gyro angle to goal based on Limelight TX
public class PassingTarget extends Command {
    private Drivetrain drivetrain;
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput;
    private PIDController turnPIDController;
    private Logger logger;
    private double currentHeading;

    private double setpoint;
    private double initialTime, currentTime;

    public PassingTarget() {
        drivetrain = Drivetrain.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kSnapToSpeakerP, LimelightConstants.kSnapToSpeakerI,
                LimelightConstants.kSnapToSpeakerD);
        turnPIDController.enableContinuousInput(-180, 180);
                turnPIDController.setIZone(4.0);

        turnFF = LimelightConstants.kSnapToSpeakerFF;
        turnThreshold = LimelightConstants.kTargetThreshold;
        turnInput = 0;
        logger = Logger.getInstance();

        currentHeading = drivetrain.getAdjustedGyroHeading();
        setpoint = 0;
        initialTime = 0.0;
        currentTime = 0.0;

        SmartDashboard.putNumber("Red Passing Target Angle", LimelightConstants.kRedCornerPassingGyroAngle);
        SmartDashboard.putNumber("Blue Passing Target Angle",LimelightConstants.kBlueCornerPassingGyroAngle);

        SmartDashboard.putNumber("Passing Target P", LimelightConstants.kSnapToSpeakerP);
        SmartDashboard.putNumber("Passing Target I", LimelightConstants.kSnapToSpeakerI);
        SmartDashboard.putNumber("Passing Target D", LimelightConstants.kSnapToSpeakerD);
        SmartDashboard.putNumber("Passing Target FF", LimelightConstants.kSnapToSpeakerFF);
        SmartDashboard.putNumber("Passing Target Threshold", LimelightConstants.kTargetThreshold);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Passing Target Command", true);

        if(DriverStation.getAlliance().get() == Alliance.Red){
            setpoint = SmartDashboard.getNumber("Red Passing Target Angle",0); 
        } else {
            setpoint = SmartDashboard.getNumber("Blue Passing Target Angle", 0);
        }

        turnPIDController.setSetpoint(setpoint);

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        turnPIDController.setP(SmartDashboard.getNumber("Passing Target P", LimelightConstants.kSnapToSpeakerP));
        turnPIDController.setI(SmartDashboard.getNumber("Passing Target I", LimelightConstants.kSnapToSpeakerI));
        turnPIDController.setD(SmartDashboard.getNumber("Passing Target D", LimelightConstants.kSnapToSpeakerD));
        turnFF = SmartDashboard.getNumber("Passing Target FF", LimelightConstants.kSnapToSpeakerFF);
        turnThreshold = SmartDashboard.getNumber("Passing Target Threshold", LimelightConstants.kTargetThreshold);

        currentTime = Timer.getFPGATimestamp();
        currentHeading = drivetrain.getAdjustedGyroHeading();
        error = currentHeading - setpoint;

        if (error < -turnThreshold) {
            turnInput = turnPIDController.calculate(currentHeading) + turnFF;
        } else if (error > turnThreshold) {
            turnInput = turnPIDController.calculate(currentHeading) - turnFF;
        } else {
            turnInput = 0;
        }

        drivetrain.drive(oi.getSwerveTranslation(), -turnInput, true, oi.getCenterOfRotation());
    }

    @Override
    public void end(boolean interrupted) {
        logger.logEvent("Passing Target Command Angle: " + currentHeading, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold;
    }
}