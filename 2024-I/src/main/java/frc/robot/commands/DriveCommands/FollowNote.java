package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.LimelightConstants;

public class FollowNote extends Command {
    private Drivetrain drivetrain;
    private LimelightIntake limelightIntake;
    private DriverOI oi;
    private double targetAngle;
    private double targetThreshold;

    private double FF;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    //follow a note in Tele-Op
    public FollowNote() {
        drivetrain = Drivetrain.getInstance();
        limelightIntake = LimelightIntake.getInstance();
        
        error = 0.0;
        thetaController = new PIDController(LimelightConstants.kFollowNoteTurnP, LimelightConstants.kFollowNoteTurnI, 
            LimelightConstants.kFollowNoteTurnD);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        
        addRequirements(drivetrain);
    
        SmartDashboard.putNumber("Follow Note P", LimelightConstants.kFollowNoteTurnP);
        SmartDashboard.putNumber("Follow Note I", LimelightConstants.kFollowNoteTurnI);
        SmartDashboard.putNumber("Follow Note D", LimelightConstants.kFollowNoteTurnD);
        SmartDashboard.putNumber("Follow Note FF", LimelightConstants.kFollowNoteTurnFF);
        SmartDashboard.putNumber("Follow Note Threshold", LimelightConstants.kFollowNoteAngleThreshold);
   }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightIntake.setPipeline(LimelightConstants.kIntakeNotePipeline); 

        thetaController = new PIDController(
            SmartDashboard.getNumber("Follow Note P", LimelightConstants.kFollowNoteTurnP),
            SmartDashboard.getNumber("Follow Note I", LimelightConstants.kFollowNoteTurnI),
            SmartDashboard.getNumber("Follow Note D", LimelightConstants.kFollowNoteTurnD)
        );
        FF = SmartDashboard.getNumber("Follow Note FF", LimelightConstants.kFollowNoteTurnFF);

        targetThreshold = SmartDashboard.getNumber("Follow Note Threshold", LimelightConstants.kFollowNoteAngleThreshold);
    }

    @Override
    public void execute() {
        double throttle = oi.getSwerveTranslation().getX();
        Translation2d position = new Translation2d(-throttle, 0.0);

        double llTurn = 0;
        if (limelightIntake.hasTarget()) {
            currentAngle = limelightIntake.getTxAverage();
            error = currentAngle - targetAngle;
            if (error < -targetThreshold)
                llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
            else if (error > targetThreshold)
                llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
        }

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
}