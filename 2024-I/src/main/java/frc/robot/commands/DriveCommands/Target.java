package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.LimelightConstants;

//Turn to target using PID
public class Target extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;
    private DriverOI oi;
    private Lights lights;

    private double error, turnThreshold, turnFF, turnInput;
    private PIDController turnPIDController;
    private Logger logger;

    public Target() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
                LimelightConstants.kTargetD);
        turnPIDController.setIZone(LimelightConstants.kTargetIZone);

        turnFF = LimelightConstants.kTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;
        logger = Logger.getInstance();
        lights = Lights.getInstance();

        addRequirements(drivetrain);
        // SmartDashboard.putNumber("Target P", LimelightConstants.kTargetP);
        // SmartDashboard.putNumber("Target I", LimelightConstants.kTargetI);
        // SmartDashboard.putNumber("Target D", LimelightConstants.kTargetD);
        // SmartDashboard.putNumber("Target FF", LimelightConstants.kTargetFF);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        logger.logEvent("Target Command", true);
        limelightShooter.setPipeline(LimelightConstants.kShooterPipeline);

        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
        } else {
            LimelightShooter.getInstance().setPriorityTag(7);
        }
    }

    @Override
    public void execute() {
        // turnPIDController.setP(SmartDashboard.getNumber("Target P", 0));
        // turnPIDController.setI(SmartDashboard.getNumber("Target I", 0));
        // turnPIDController.setD(SmartDashboard.getNumber("Target D", 0));
        // turnFF = (SmartDashboard.getNumber("Target FF", 0));

        if(limelightShooter.hasTarget()){
            if (Math.abs(error) >= 3 * turnThreshold)
                lights.requestState(LightState.HAS_TARGET);
            else
                lights.requestState(LightState.TARGETED);
        } 

        if (limelightShooter.hasTarget()) {
            error = limelightShooter.getTxAverage() - LimelightConstants.kTargetTarget;
            if (error < -turnThreshold) {
                turnInput = turnPIDController.calculate(error) + turnFF;
            } else if (error > turnThreshold) {
                turnInput = turnPIDController.calculate(error) - turnFF;
            } else {
                turnInput = 0;
            }
        } else {
            turnInput = 0;
        }
        drivetrain.drive(oi.getSwerveTranslation(), turnInput, true, oi.getCenterOfRotation());
    }

    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putBoolean("Targetting", false);
        drivetrain.stop();
        logger.logEvent("Target Command", false);
        lights.requestState(LightState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}