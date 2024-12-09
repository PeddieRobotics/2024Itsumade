package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class RunFlywheel extends Command {
    private Flywheel flywheel;


    public RunFlywheel(){
        flywheel = Flywheel.getInstance();
    }


    @Override
    public void initialize(){
        flywheel.runBlackFlywheelVelocitySetpoint(30);
        flywheel.runOrangeFlywheelVelocitySetpoint(30);
    }

    @Override
    public void execute(){
        flywheel.runBlackFlywheelVelocitySetpoint(30);
        flywheel.runOrangeFlywheelVelocitySetpoint(30);
    }

    @Override
    public void end(boolean interrupted){
        flywheel.runBlackFlywheelVelocitySetpoint(0);
        flywheel.runOrangeFlywheelVelocitySetpoint(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
