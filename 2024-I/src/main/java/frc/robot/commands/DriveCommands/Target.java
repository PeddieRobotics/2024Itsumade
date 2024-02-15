package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.LimelightConstants;

public class Target extends Command {
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter; // TODO: figure out front or back LL
    private DriverOI oi;

    private double error, turnThreshold, turnFF, turnInput;
    private PIDController turnPIDController;

    public Target() {
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        turnPIDController = new PIDController(LimelightConstants.kTargetP, LimelightConstants.kTargetI,
                LimelightConstants.kTargetD);
        turnFF = LimelightConstants.kTargetFF;
        turnThreshold = LimelightConstants.kTargetAngleThreshold;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        limelightShooter.setPipeline(0);
    }

    @Override
    public void execute() {
        turnPIDController.setP(SmartDashboard.getNumber("Target P", 0));
        turnPIDController.setP(SmartDashboard.getNumber("Target I", 0));
        turnPIDController.setP(SmartDashboard.getNumber("Target D", 0));
        turnFF = (SmartDashboard.getNumber("Target FF", 0));

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
        drivetrain.drive(oi.getSwerveTranslation(), error, true, oi.getCenterOfRotation());

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}