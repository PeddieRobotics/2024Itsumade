package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utils.OperatorOI;

public class ManualClimberControl extends Command{
    private Climber climber;
    private OperatorOI operatorOI;

    //blue coordinate system, give input
    public ManualClimberControl(){
        climber = Climber.getInstance();
    }

    @Override
    public void initialize(){
        operatorOI = OperatorOI.getInstance();
    }

    @Override 
    public void execute(){
        climber.runLeftMotor(operatorOI.getLeftForward());
        climber.runRigthMotor(operatorOI.getRightForward());
    }

    @Override
    public void end(boolean interrupted){
        climber.stopClimber();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}