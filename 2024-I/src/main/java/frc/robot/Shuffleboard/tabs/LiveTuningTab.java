package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

public class LiveTuningTab extends ShuffleboardTabBase{
    private ComplexWidget autoChooserWidget, cameraWidget;

    private Arm arm;
    private Autonomous autonomous;
    private Flywheel flywheel;
    private Hopper hopper;
    private Intake intake;
    private Superstructure superstructure;

    private GenericEntry stateEntry, armAngleEntry,
    flywheelAtRPMEntry, armDeltaEntry, flywheelDeltaEntry, flywheelLeftRPMEntry, 
    flywheelRightRPMEntry, isIndexedOverrideEntry, 
    isGamePieceIndexedEntry, stowAfterShootOverrideEntry, topSensorEntry, bottomSensorEntry,
    hasGamePieceEntry;

    private ComplexWidget mAutoChooser;

    //Sendable Chooser
    private static SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String, Command> autoRoutines;

    public OperatorTab(){
        arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        superstructure = Superstructure.getInstance();
        autoRoutineSelector = AutoBuilder.buildAutoChooser();
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        autoRoutineSelector = new SendableChooser<Command>();
        SmartDashboard.putData("Auto Chooser", autoRoutineSelector);

        try{ 
            stateEntry = tab.add("State", "STOW")
            .withSize(1,1)
            .withPosition(0,1)
            .getEntry();
        } catch(IllegalArgumentException e){}
    }

}
