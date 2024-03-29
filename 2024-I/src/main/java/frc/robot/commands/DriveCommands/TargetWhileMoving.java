package frc.robot.commands.DriveCommands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class TargetWhileMoving extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter;

    private double turnThreshold, turnFF;
    private PIDController turnPIDController;
    private Logger logger;

    public TargetWhileMoving() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
                LimelightConstants.kTargetD);
        turnPIDController.setIZone(4.0);

        turnFF = LimelightConstants.kTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;
        logger = Logger.getInstance();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        logger.logEvent("Target While Moving Command", true);
        limelightShooter.setPipeline(LimelightConstants.kShooterPipeline);

        if(DriverStation.getAlliance().get() == Alliance.Red){
            LimelightShooter.getInstance().setPriorityTag(4);
        } else {
            LimelightShooter.getInstance().setPriorityTag(7);
        }

        PPHolonomicDriveController.setRotationTargetOverride(this::calculateRotation);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        logger.logEvent("Target While Moving Command", false);
        PPHolonomicDriveController.setRotationTargetOverride(this::resetRotation);
    }

    @Override
    public boolean isFinished() {
        // FIX THIS!
        return false;
    }

    private Optional<Rotation2d> calculateRotation() {
        double angle = 0;
        if (limelightShooter.hasTarget()) {
            double error = limelightShooter.getTxAverage();
            if (error < -turnThreshold)
                angle = turnPIDController.calculate(error) + turnFF;
            else if (error > turnThreshold)
                angle = turnPIDController.calculate(error) - turnFF;
        }
        return Optional.of(Rotation2d.fromDegrees(angle));
    }

    private Optional<Rotation2d> resetRotation() {
        return Optional.empty();
    }
}