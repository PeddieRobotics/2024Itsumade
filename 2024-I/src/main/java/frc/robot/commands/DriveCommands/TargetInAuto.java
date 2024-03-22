package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    private double error, turnThreshold, turnFF, turnInput, initialTime, currentTime;
    private PIDController turnPIDController;
    private Logger logger;

    public TargetInAuto() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        lights = Lights.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
                LimelightConstants.kTargetD);
        turnPIDController.setIZone(4.0);

        turnFF = LimelightConstants.kTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        turnInput = 0;
        logger = Logger.getInstance();

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
            error = limelightShooter.getTxAverage();
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

        drivetrain.drive(oi.getSwerveTranslation(), turnInput * 2, true, oi.getCenterOfRotation());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Target Command", false);
        lights.requestState(LightState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < turnThreshold || (currentTime - initialTime > 1.0);
    }
}