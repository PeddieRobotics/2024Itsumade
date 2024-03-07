package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.AutoConstants;

public class LayupPrep extends Command{

    private Superstructure superstructure;
    private Arm arm;
    private double initialTime;

    public LayupPrep(){
        superstructure = Superstructure.getInstance();
        arm = Arm.getInstance();
        addRequirements(superstructure, arm);
    }

    

    @Override
    public void initialize(){
        superstructure.requestState(SuperstructureState.FRONT_LAYUP_PREP);
        initialTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return (arm.isAtFrontLayupAngle() && Timer.getFPGATimestamp() - initialTime > 0.1) || Timer.getFPGATimestamp() - initialTime > AutoConstants.kLayupPrepDeadlineTime;
    }
}