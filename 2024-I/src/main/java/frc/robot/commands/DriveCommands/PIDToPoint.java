package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
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

public class PIDToPoint extends Command{
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter; // TODO: figure out front or back LL
    private DriverOI oi;

    //turn
    private double turnThreshold, turnFF, turnTarget;
    private PIDController turnController;

    //translate
    private double moveThreshold, moveFF, xTarget, yTarget;
    private PIDController xController, yController;
    
    //blue coordinate system, give input
    public PIDToPoint(double x, double y, double theta){
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        turnController = new PIDController(Constants.LimelightConstants.kDriveToTargetTurnP, Constants.LimelightConstants.kDriveToTargetTurnI, 
            Constants.LimelightConstants.kDriveToTargetTurnD);
        turnController.setIZone(Constants.LimelightConstants.kDriveToTargetIZone);
        turnController.enableContinuousInput(-180, 180);
        turnThreshold = Constants.LimelightConstants.kDriveToTargetTurnThreshold; 
        turnFF = Constants.LimelightConstants.kDriveToTargetTurnFF; 
        turnTarget = theta;

        xController = new PIDController(Constants.LimelightConstants.kDriveToTargetMoveP, Constants.LimelightConstants.kDriveToTargetMoveI, 
            Constants.LimelightConstants.kDriveToTargetMoveD);
        xTarget = x;

        yController = new PIDController(Constants.LimelightConstants.kDriveToTargetMoveP, Constants.LimelightConstants.kDriveToTargetMoveI, 
            Constants.LimelightConstants.kDriveToTargetMoveD);
        yTarget = y;

        moveThreshold = Constants.LimelightConstants.kDriveToTargetMoveThreshold; 
        moveFF = Constants.LimelightConstants.kDriveToTargetMoveFF; 

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        oi = DriverOI.getInstance();
        if(DriverStation.getAlliance().get() == Alliance.Red){
            limelightShooter.setPriorityTag(5);
        } else {
            limelightShooter.setPriorityTag(6);
        }
    }

    @Override 
    public void execute(){
        Translation2d position = new Translation2d();
        double turnAngle = 0.0, xMove = 0.0, yMove = 0.0;

        if (limelightShooter.hasTarget()) {
            double currentAngle = limelightShooter.getRotationAverage();
            double currentTX = limelightShooter.getRXAverage(); 
            double currentTY = limelightShooter.getRYAverage(); 

            double turnError = currentAngle - turnTarget;
            double xError = currentTX - xTarget; 
            double yError = currentTY - yTarget; 

            if (turnError < -turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) + turnFF;
            else if (turnError > turnThreshold)
                turnAngle = turnController.calculate(currentAngle, turnTarget) - turnFF;

            if (xError < -moveThreshold)
                xMove = xController.calculate(currentTX, xTarget) + moveFF; 
            else if (xError > moveThreshold)
                xMove = xController.calculate(currentTX, xTarget) - moveFF; 

            if (yError < -moveThreshold)
                yMove = yController.calculate(currentTY, yTarget) + moveFF; 
            else if (yError > moveThreshold)
                yMove = yController.calculate(currentTY, yTarget) - moveFF; 

            SmartDashboard.putNumber("x move", xMove);
            SmartDashboard.putNumber("y move", yMove);
            SmartDashboard.putNumber("tx avg", currentTX);
            SmartDashboard.putNumber("ty avg", currentTY);
            SmartDashboard.putNumber("x error", xError);
            SmartDashboard.putNumber("y error", yError);

            position = new Translation2d(xMove, yMove); 
        }

        drivetrain.drive(position, turnAngle, false, new Translation2d());
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
          } else {
            LimelightShooter.getInstance().setPriorityTag(7);
          }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}