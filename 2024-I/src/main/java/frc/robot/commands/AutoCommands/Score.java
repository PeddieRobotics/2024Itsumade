package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.AutoConstants;

public class Score extends Command {

    private Superstructure superstructure;
    private Hopper hopper;
    private double initialTime;

    public Score() {
        superstructure = Superstructure.getInstance();
        hopper = Hopper.getInstance();

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        superstructure.sendToScore();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (!hopper.getTopSensor()) || Timer.getFPGATimestamp() - initialTime > AutoConstants.kScoreDeadlineTime;
    }
}