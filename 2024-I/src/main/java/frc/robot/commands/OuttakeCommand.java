package frc.robot.commands;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeCommand extends Command {
    private Superstructure superstructure;

    public OuttakeCommand() {
        superstructure = Superstructure.getInstance();
    }

    @Override
    public void initialize() {
        superstructure.requestState(SuperstructureState.OUTTAKE);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.requestState(SuperstructureState.GROUND_INTAKE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
