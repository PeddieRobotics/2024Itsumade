package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.utils.OperatorOI;

public class ManualArmControl extends Command{
    private Arm arm;
    private OperatorOI oi;

    private double currentArmAngle;

    public ManualArmControl() {
        arm = Arm.getInstance();
        oi = OperatorOI.getInstance();
    }
    @Override
    public void initialize() {
        currentArmAngle = arm.getAbsoluteCANCoderPosition();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setArmPercentOutput(oi.getForward()/10);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
}
