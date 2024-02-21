package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;

public class DeployClimber extends Command{
   
    private Climber climber;
    
    //blue coordinate system, give input
    public DeployClimber(){
        climber = Climber.getInstance();
        
    }

    

    @Override
    public void initialize(){
       
    }

    @Override 
    public void execute(){
        climber.deployClimber();
    }

    @Override
    public void end(boolean interrupted){
      
    }

    @Override
    public boolean isFinished(){
       return climber.isClimberDeployed();
    }
}