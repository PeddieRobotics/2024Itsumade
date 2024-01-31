package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class Autonomous extends SubsystemBase {
    private static Autonomous instance;
    private static SendableChooser<Command> autoChooser;
    private Superstructure superstructure;

    public static Autonomous getInstance(){
        if (instance == null){
            instance = new Autonomous();
        }
        return instance;
    }

    public Autonomous(){
        NamedCommands.registerCommand("Intake", new InstantCommand( () -> {superstructure.requestState(SuperstructureState.GROUND_INTAKE);} ) );
        NamedCommands.registerCommand("SideLayup", new InstantCommand( () -> {superstructure.shoot(Constants.FlywheelConstants.SideLayupFlywheelSpeed);} ) );
        NamedCommands.registerCommand("FrontLayup", new InstantCommand( () -> {superstructure.shoot(Constants.FlywheelConstants.FrontLayupFlywheelSpeed);} ) );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}
