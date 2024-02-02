package frc.robot.subsystems;

import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase {
    private static Autonomous instance;
    private static SendableChooser<Command> autoChooser;

    public static Autonomous getInstance(){
        if (instance == null){
            instance = new Autonomous();
        }
        return instance;
    }

    public Autonomous(){
        NamedCommands.registerCommand("Command", new InstantCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }

    //EDIT THIS METHOD LATER, FILLER METHOD FOR THE OPERATOR TAB THAT NEEDS ATTENTION!!!
    public Hashtable<String, Command> getAutoRoutines() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAutoRoutines'");
    }
}
