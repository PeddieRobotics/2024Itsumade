package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;

public class FollowNote extends Command {
    private Drivetrain drivetrain;
    private LimelightBack limelightBack;
    private DriverOI oi;
    private double angleThreshold;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    //follow a note in Tele-Op
    public FollowNote() {
        drivetrain = Drivetrain.getInstance();
        limelightBack = LimelightBack.getInstance();
        
        angleThreshold = Constants.LimelightConstants.kFollowNoteAngleThreshold;

        error = 0.0;
        thetaController = new PIDController(Constants.LimelightConstants.kFollowNoteTurnP, Constants.LimelightConstants.kFollowNoteTurnI, 
            Constants.LimelightConstants.kFollowNoteTurnD);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        llTurn = 0;
        
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightBack.setPipeline(1); 
    }

    @Override
    public void execute() {
        double throttle = oi.getSwerveTranslation().getX();
        Translation2d position = new Translation2d(-throttle, 0.0);

        if (limelightBack.hasTarget()) {
            currentAngle = limelightBack.getTxAverage();
            error = currentAngle - targetAngle;
            SmartDashboard.putNumber("DATA: Error", currentAngle);
            if (error < -angleThreshold) {
                llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
            } else if (error > angleThreshold) {
                llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
            }

            else {
                llTurn = 0;
            }
        } else {
            llTurn = 0;
        }

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
        SmartDashboard.putNumber("DATA: Note Tx", currentAngle);
        SmartDashboard.putNumber("DATA: Note Turn", llTurn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
        // return Math.abs(limelightFront.getTxAverage() - targetAngle) < angleThreshold;
    }
}