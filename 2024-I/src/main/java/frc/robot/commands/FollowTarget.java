package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class FollowTarget extends Command {
    private final Limelight limelight;

    public FollowTarget(){
        limelight = Limelight.getInstance();

        addRequirements(limelight);
    }
}
