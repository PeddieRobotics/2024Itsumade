package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RetractClimber extends Command{
   
    private Climber climber;
    
    //blue coordinate system, give input
    public RetractClimber(){
        climber = Climber.getInstance();
        
    }

    

    @Override
    public void initialize(){
       
    }

    @Override 
    public void execute(){
        climber.retractClimber();
    }

    @Override
    public void end(boolean interrupted){
      
    }

    @Override
    public boolean isFinished(){
       return climber.isDoneClimbing();
    }
}