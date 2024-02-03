package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;

public class DriveToPoint extends Command{
    private Drivetrain drivetrain;
    private Limelight limelightBack; // TODO: figure out front or back LL
    
    //blue coordinate system, give input
    public DriveToPoint(double x, double y, double theta){
        drivetrain = Drivetrain.getInstance();
        limelightBack = LimelightBack.getInstance();
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
