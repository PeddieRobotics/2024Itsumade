package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;

public class Target extends Command{
    private Drivetrain drivetrain;
    private LimelightShooter limelightShooter; // TODO: figure out front or back LL
    private DriverOI oi;

    //turn
    private double turnThreshold, turnFF, turnTarget;
    private PIDController turnController;

    //translate
    private double moveThreshold, moveFF, xTarget, yTarget;
    private PIDController xController, yController;
    
    //blue coordinate system, give input
    public Target(double x, double y, double theta){
        drivetrain = Drivetrain.getInstance();
        limelightShooter = LimelightShooter.getInstance();

        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        oi = DriverOI.getInstance();

        //TODO: update
        limelightShooter.setPipeline(0);
    }

    @Override 
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}