package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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

//Turn to target using PID
public class RotateToAngle extends Command {
    private Drivetrain drivetrain;
    private DriverOI oi;

    private double currentHeading, targetAngle, initialAngle;
    private PIDController turnPIDController;

    public RotateToAngle(double targetAngle) {
        drivetrain = Drivetrain.getInstance();
        this.targetAngle = targetAngle;

        turnPIDController = new PIDController(0.005, 0.00001, 0);
        turnPIDController.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oi = DriverOI.getInstance();
        initialAngle = drivetrain.getHeading();
        turnPIDController.reset();
    }

    @Override
    public void execute() {
        double turn = 0.0;
        double turnFF = 0.05;
        currentHeading = drivetrain.getHeading();

        turn = turnPIDController.calculate(currentHeading, targetAngle);

        drivetrain.drive(oi.getSwerveTranslation(), turn + turnFF * Math.signum(turn), true, new Translation2d(0,0));
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(currentHeading) - targetAngle) < 1.5;
    }
}