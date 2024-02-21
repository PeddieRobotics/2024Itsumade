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