package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.utils.Logger;

public class DeployClimber extends Command{
   
    private Climber climber;
    private Lights lights;
    
    public DeployClimber(){
        lights = Lights.getInstance();
        climber = Climber.getInstance();
        addRequirements(climber);
    }

    

    @Override
    public void initialize(){
       Logger.getInstance().logEvent("Deploy Climber", true);
       lights.requestState(LightState.CLIMBING);
    }

    @Override 
    public void execute(){
        climber.deployClimber();
    }

    @Override
    public void end(boolean interrupted){
        Logger.getInstance().logEvent("Deploy Climber", false);
    }

    @Override
    public boolean isFinished(){
       return climber.leftArmDeployed() && climber.rightArmDeployed();
    }
}