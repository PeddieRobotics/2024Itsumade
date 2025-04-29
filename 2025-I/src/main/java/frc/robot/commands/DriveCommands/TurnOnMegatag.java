package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

//TODO: can probably turn into instantComamnd
public class TurnOnMegatag extends Command {
    private Drivetrain drivetrain;

    public TurnOnMegatag() {
        drivetrain = Drivetrain.getInstance();
   }

    @Override
    public void initialize() {
        drivetrain.setUseMegaTag(true);
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