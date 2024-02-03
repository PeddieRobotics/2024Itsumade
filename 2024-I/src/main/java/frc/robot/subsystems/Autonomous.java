package frc.robot.subsystems;

import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;

public class Autonomous extends SubsystemBase {
    private static SendableChooser<Command> autoChooser;
    private Superstructure superstructure;

    private static Autonomous autonomous;

    public Autonomous(){
        NamedCommands.registerCommand("Intake", new InstantCommand( () -> {superstructure.requestState(SuperstructureState.GROUND_INTAKE);} ) );
        // NamedCommands.registerCommand("SideLayup", new InstantCommand( () -> {superstructure.shoot(Constants.FlywheelConstants.SideLayupFlywheelSpeed);} ) );
        // NamedCommands.registerCommand("FrontLayup", new InstantCommand( () -> {superstructure.shoot(Constants.FlywheelConstants.FrontLayupFlywheelSpeed);} ) );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

  public static Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public static Autonomous getInstance(){
    if(autonomous == null){
      autonomous = new Autonomous();
    }return autonomous;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
