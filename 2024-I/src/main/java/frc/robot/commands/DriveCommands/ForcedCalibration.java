package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;


//forced calibration used in auto, quickly sets std very small back to default
public class ForcedCalibration extends Command {
    private Drivetrain drivetrain;
    private int cycles = 0;
    private static final int maxCycles = 2;
    private boolean wasUsingMegaTag;

    public ForcedCalibration() {
        drivetrain = Drivetrain.getInstance();
   }

    @Override
    public void initialize() {
        wasUsingMegaTag = drivetrain.getUseMegaTag();
        if (!wasUsingMegaTag)
            drivetrain.setUseMegaTag(true);
        drivetrain.setIsForcingCalibration(true);
        cycles = 0;
    }

    @Override
    public void execute() {
        cycles++;
    }

    @Override
    public void end(boolean interrupted) {
        if (!wasUsingMegaTag)
            drivetrain.setUseMegaTag(false);
        drivetrain.setUseMegaTag(false);
        drivetrain.setIsForcingCalibration(false);
    }

    @Override
    public boolean isFinished() {
        return cycles >= maxCycles;
    }
}