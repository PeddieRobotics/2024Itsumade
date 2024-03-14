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

public class HybridTarget extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput;
    // Odometry Target Values
    private double speakerPoseX, speakerPoseY ,targetAngle, currentAngle;
    private Pose2d currentOdometry;

    private PIDController turnPIDController;
    private Logger logger;
    
    public HybridTarget() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        logger = Logger.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
        LimelightConstants.kTargetD);
        
        turnFF = LimelightConstants.kTargetFF;
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
        logger.logEvent("Started Hybrid Target Command", true);

        if(DriverStation.getAlliance().get() == Alliance.Red){
            speakerPoseX = LimelightConstants.kRedSpeakerPositionX;
            speakerPoseY = LimelightConstants.kRedSpeakerPositionY;
        }
        else{
            speakerPoseX = LimelightConstants.kBlueSpeakerPositionX;
            speakerPoseY = LimelightConstants.kBlueSpeakerPositionY;
        }
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}