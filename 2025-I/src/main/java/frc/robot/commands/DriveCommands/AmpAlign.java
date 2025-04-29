package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

public class AmpAlign extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;

    private double odometryAngleError, txError, odometryAngleThreshold, horizontalThreshold, horizontalFF, odometryTurnFF, turnInput, horizontalInput;
    
    // Odometry Target Values
    private double targetAngle, currentAngle;

    private Pose2d currentOdometry;

    private PIDController horizontalAlignPIDController, odometryTurnPIDController;

    private boolean hasRotated, acquiredTarget;

    private Logger logger;

    public AmpAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        logger = Logger.getInstance();

        horizontalAlignPIDController = new PIDController(LimelightConstants.kHorizontalAlignP, LimelightConstants.kHorizontalAlignI,
                LimelightConstants.kHorizontalAlignD);
        odometryTurnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI, LimelightConstants.kOdometryTargetD);
        odometryTurnPIDController.enableContinuousInput(-180, 180);
        horizontalFF = LimelightConstants.kHorizontalAlignFF;
        odometryTurnFF = LimelightConstants.kOdometryTargetFF;

        odometryAngleThreshold = LimelightConstants.kAmpAlignAngleThreshold;
        horizontalThreshold = LimelightConstants.kAmpAlignAngleThreshold;

        turnInput = 0.0;
        horizontalInput = 0.0;

        currentOdometry = drivetrain.getPose();
        currentAngle = 0.0;
        odometryAngleError = 0.0;
        txError = 0.0;
        hasRotated = false;
        acquiredTarget = false;

        targetAngle = LimelightConstants.kAmpOdometryHeading;

        SmartDashboard.putNumber("Horizontal Align P", LimelightConstants.kHorizontalAlignP);
        SmartDashboard.putNumber("Horizontal Align I", LimelightConstants.kHorizontalAlignI);
        SmartDashboard.putNumber("Horizontal Align D", LimelightConstants.kHorizontalAlignD);
        SmartDashboard.putNumber("Horizontal Align FF", LimelightConstants.kHorizontalAlignFF);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Amp Align Command", true);
        if(DriverStation.getAlliance().get() == Alliance.Red){
            limelightShooter.setPriorityTag(5);
        } else {
            limelightShooter.setPriorityTag(6);
        }
        hasRotated = false;
        acquiredTarget = false;

        txError = limelightShooter.getTxAverage();
    }

    @Override
    public void execute() {
        currentOdometry = drivetrain.getPose();
        currentAngle = currentOdometry.getRotation().getDegrees();
        
        odometryAngleError = currentAngle - targetAngle;
        txError = limelightShooter.getTxAverage();

        if (odometryAngleError < -odometryAngleThreshold) {
            turnInput = odometryTurnPIDController.calculate(odometryAngleError) + odometryTurnFF;
        } else if (odometryAngleError > odometryAngleThreshold) {
            turnInput = odometryTurnPIDController.calculate(odometryAngleError) - odometryTurnFF;
        } else{
            turnInput = 0;
            hasRotated = true;
        }

        if(hasRotated && limelightShooter.hasTarget() && (limelightShooter.getTargetID() == 5 || limelightShooter.getTargetID() == 6) && Math.abs(txError) < horizontalThreshold){
            acquiredTarget = true;
        }

        if(!hasRotated){
            drivetrain.drive(oi.getSwerveTranslation(), turnInput, true, oi.getCenterOfRotation());
        }
        else{
            if(limelightShooter.hasTarget() && Math.abs(txError) < 25.0){
                if (txError < -horizontalThreshold) {
                    horizontalInput = horizontalAlignPIDController.calculate(txError) + horizontalFF;
                } else if (txError > horizontalThreshold) {
                    horizontalInput = horizontalAlignPIDController.calculate(txError) - horizontalFF;
                } else{
                    horizontalInput = 0;
                }

                if(DriverStation.getAlliance().get() == Alliance.Red){
                    horizontalInput = -horizontalInput;
                } 

            } else{
                if(acquiredTarget){
                    horizontalInput = 0;
                }
                else{
                    horizontalInput = oi.getSwerveTranslation().getX();
                }
            }

            drivetrain.drive(new Translation2d(horizontalInput, oi.getSwerveTranslation().getY()), turnInput, true, oi.getCenterOfRotation());
        }

    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Amp Align Command", false);
        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
        } else {
            LimelightShooter.getInstance().setPriorityTag(8);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // hasRotated && (Math.abs(txError) < horizontalThreshold);
    }
}