package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class RunFlywheel extends Command {
    private Flywheel flywheel;
    private Superstructure superstructure;

    public RunFlywheel(){
        flywheel = Flywheel.getInstance();
        superstructure = Superstructure.getInstance();
    }


    @Override
    public void initialize(){
        superstructure.requestState(SuperstructureState.INTAKING);
    }

    @Override
    public void execute(){
       
    }

    @Override
    public void end(boolean interrupted){
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
