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

public class HybridAmpAlign extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;

    private double odometryError, txError, odometryThreshold, horizontalThreshold, horizontalFF, odometryTurnFF, turnInput, horizontalInput;
    // Odometry Target Values
    private double targetAngle, currentAngle;
    private Pose2d currentOdometry;

    private PIDController horizontalAlignPIDController, odometryTurnPIDController;

    private boolean hasRotated;

    private Logger logger;

    public HybridAmpAlign() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        logger = Logger.getInstance();

        horizontalAlignPIDController = new PIDController(LimelightConstants.kHorizontalAlignP, LimelightConstants.kHorizontalAlignI,
                LimelightConstants.kHorizontalAlignD);
        odometryTurnPIDController = new PIDController(LimelightConstants.kOdometryTargetP, LimelightConstants.kOdometryTargetI, LimelightConstants.kOdometryTargetD);

        horizontalFF = LimelightConstants.kHorizontalAlignFF;
        odometryTurnFF = LimelightConstants.kOdometryTargetFF;
        odometryThreshold = LimelightConstants.kAmpAlignAngleThreshold;
        horizontalThreshold = LimelightConstants.kAmpAlignAngleThreshold;
        turnInput = 0;
        horizontalInput = 0.0;
        currentOdometry = drivetrain.getPose();
        currentAngle = 0.0;
        odometryError = 0.0;
        txError = 0.0;
        hasRotated = false;

        targetAngle = LimelightConstants.kAmpOdometryHeading;

        SmartDashboard.putNumber("Horizontal Align P", LimelightConstants.kHorizontalAlignP);
        SmartDashboard.putNumber("Horizontal Align I", LimelightConstants.kHorizontalAlignI);
        SmartDashboard.putNumber("Horizontal Align D", LimelightConstants.kHorizontalAlignD);
        SmartDashboard.putNumber("Horizontal Align FF", LimelightConstants.kHorizontalAlignFF);
        SmartDashboard.putNumber("Horizontal Align Input", horizontalInput);

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
        hasRotated = true;

        txError = limelightShooter.getTxAverage();
    }

    @Override
    public void execute() {
        //update pid values
        horizontalAlignPIDController.setP(SmartDashboard.getNumber("Horizontal Align P", 0));
        horizontalAlignPIDController.setI(SmartDashboard.getNumber("Horizontal Align I", 0));
        horizontalAlignPIDController.setD(SmartDashboard.getNumber("Horizontal Align D", 0));
        horizontalFF = SmartDashboard.getNumber("Horizontal Align FF", 0);
        
        currentOdometry = drivetrain.getPose();
        currentAngle = currentOdometry.getRotation().getDegrees();
        odometryError = currentAngle - targetAngle;
        txError = limelightShooter.getTxAverage();

        if(Math.abs(odometryError) > odometryThreshold && !hasRotated){ //correct rotation
            if (odometryError < -odometryThreshold) {
                turnInput = odometryTurnPIDController.calculate(odometryError) + odometryTurnFF;
            } else if (odometryError > odometryThreshold) {
                turnInput = odometryTurnPIDController.calculate(odometryError) - odometryTurnFF;
            } 
            drivetrain.drive(oi.getSwerveTranslation(), turnInput, true, oi.getCenterOfRotation());
        } else {
            hasRotated = true;
        }
        
        if (Math.abs(txError) > horizontalThreshold && hasRotated) { //use tx to fix translation
            if (txError < -horizontalThreshold) {
                horizontalInput = horizontalAlignPIDController.calculate(txError) + horizontalFF;
            } else if (txError > horizontalThreshold) {
                horizontalInput = horizontalAlignPIDController.calculate(txError) - horizontalFF;
            } 

            if(DriverStation.getAlliance().get() == Alliance.Red){
                horizontalInput = -horizontalInput;
            } 

            drivetrain.drive(new Translation2d(horizontalInput, oi.getSwerveTranslation().getY()), oi.getRotation(), true, oi.getCenterOfRotation());
        }
        else if(hasRotated){
            drivetrain.drive(new Translation2d(0, oi.getSwerveTranslation().getY()), oi.getRotation(), true, oi.getCenterOfRotation());
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