package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Logger;


//forced calibration used in auto, quickly sets std very small back to default
public class ForcedCalibration extends Command {
    private Drivetrain drivetrain;
    private int cycles = 0;
    private static final int maxCycles = 2;

    public ForcedCalibration() {
        drivetrain = Drivetrain.getInstance();
   }

    @Override
    public void initialize() {
        Logger.getInstance().logEvent("Forced calibration command", true);
        drivetrain.setIsForcingCalibration(true);
        cycles = 0;
    }

    @Override
    public void execute() {
        cycles++;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.getInstance().logEvent("Forced calibration command", false);
        drivetrain.setIsForcingCalibration(false);
    }

    @Override
    public boolean isFinished() {
        return cycles >= maxCycles;
    }
}