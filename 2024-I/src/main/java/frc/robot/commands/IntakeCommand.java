package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private Intake intake;


    
    public IntakeCommand(){
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override

    public void initialize(){
        intake.setSpeed(0.3);
    }
    @Override

    public void execute(){
        intake.setSpeed(0.3);
    }
    @Override

    public void end(boolean interrupted){
        intake.setSpeed(0);
    }
    @Override

    public boolean isFinished(){
        return false;
    }
}
