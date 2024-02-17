package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnOffMegatag extends Command {
    private Drivetrain drivetrain;

    public TurnOffMegatag() {
        drivetrain = Drivetrain.getInstance();
   }

    @Override
    public void initialize() {
        drivetrain.setUseMegaTag(false);
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