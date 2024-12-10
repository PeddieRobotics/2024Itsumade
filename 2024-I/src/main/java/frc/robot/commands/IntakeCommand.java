package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class IntakeCommand extends Command {
    private Intake intake;
    private Superstructure superstructure;

    
    public IntakeCommand(){
        intake = Intake.getInstance();
    }

    @Override

    public void initialize(){
        superstructure.requestState(SuperstructureState.SHOOTING);
    }
    @Override

    public void execute(){
    }
    @Override

    public void end(boolean interrupted){
        intake.setSpeed(0);
    }
    @Override

    public boolean isFinished(){
        return true;
    }
}
