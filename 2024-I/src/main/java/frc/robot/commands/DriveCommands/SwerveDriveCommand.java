package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverOI;

public class SwerveDriveCommand extends Command {
    private Drivetrain drivetrain;
    private DriverOI driverOI;

    public SwerveDriveCommand() {
        drivetrain = Drivetrain.getInstance();
        driverOI = DriverOI.getInstance();
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d position =  driverOI.getSwerveTranslation();
        double rotation = -driverOI.getRotation();
        Translation2d centerOfRotation = driverOI.getCenterOfRotation();

        SmartDashboard.putNumber("field relative input forward axis", position.getX());
        SmartDashboard.putNumber("field relative input strafe axis", position.getY());

        drivetrain.drive(position, rotation, true, centerOfRotation);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
