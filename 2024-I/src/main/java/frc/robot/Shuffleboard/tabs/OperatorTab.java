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

public class OperatorTab extends ShuffleboardTabBase{
    private ComplexWidget autoChooserWidget, cameraWidget;

    private Arm arm;
    private Autonomous autonomous;
    private Flywheel flywheel;
    private Hopper hopper;
    private Intake intake;
    private Superstructure superstructure;

    private GenericEntry stateEntry, armAngleEntry,
    flywheelAtRPMEntry, armDeltaEntry, flywheelDeltaEntry, flywheelLeftRPMEntry, 
    flywheelRightRPMEntry, isIndexedOverrideEntry, stowAfterShootOverrideEntry, topSensorEntry, bottomSensorEntry,
    hasGamePieceEntry;

    private ComplexWidget mAutoChooser;

    //Sendable Chooser
    private static SendableChooser<Command> autoRoutineSelector;
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

            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            cameraWidget = tab.addCamera("Camera", "LL Intake", "http://10.58.95.53:5800")
            .withSize(5,5)
            .withPosition(5,1);

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1,1)
            .withPosition(1, 2)
            .getEntry();

            armDeltaEntry = tab.add("Arm Delta", 0.0)
            .withSize(1,1)
            .withPosition(1, 2)
            .getEntry();

            flywheelLeftRPMEntry = tab.add("Flywheel Left RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 0)
            .getEntry();

            flywheelRightRPMEntry = tab.add("Flywheel Right RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 1)
            .getEntry();

            flywheelIsAtRPMEntry = tab.add("Flywheel At RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 1)
            .getEntry();

            // Return to this later
            // isIndexedOverrideEntry = tab.add("Piece Indexed Override", false)
            // .withWidget(BuiltInWidgets.kToggleButton)
            // .withSize(2,1)
            // .withPosition(3, 0)
            // .getEntry();

            stowAfterShootOverrideEntry = tab.add("Stow After Shoot Override", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2,2)
            .withPosition(0, 7)
            .getEntry();

            topSensorEntry = tab.add("Top Sensor?", false)
            .withSize(1,1)
            .withPosition(0, 2)
            .getEntry();

            bottomSensorEntry = tab.add("Bottom Sensor?", false)
            .withSize(1,1)
            .withPosition(0, 3)
            .getEntry();   

            hasGamePieceEntry = tab.add("Has Gamepiece?", false)
            .withSize(2,2)
            .withPosition(3, 1)
            .getEntry();            
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            armAngleEntry.setDouble(arm.getArmAngleDegrees());

            flywheelAtRPMEntry.getBoolean(flywheel.isAtRPM());
            flywheelLeftRPMEntry.setDouble(flywheel.getFlywheelLeftRPM());
            flywheelRightRPMEntry.setDouble(flywheel.getFlywheelRightRPM());
            flywheel.setRPMDelta(flywheelDeltaEntry.getDouble(0));

            arm.setArmDelta(armDeltaEntry.getDouble(0));

            // isIndexedOverrideEntry.getBoolean(false);  // Return to this later
            stateEntry.setString(superstructure.getRobotState());

            topSensorEntry.setBoolean(hopper.getTopSensor());
            bottomSensorEntry.setBoolean(hopper.getBottomSensor());

            hasGamePieceEntry.setBoolean(hopper.hasGamepiece());
        } catch(IllegalArgumentException e){}
    }

    public void setupAutoSelector(){
        SendableChooser<Command> autoChooser = autonomous.getAutoChooser();
        mAutoChooser = tab.add("Auto routine", autoChooser).withSize(5,2).withPosition(0,8); // comp settings: withPosition(16,1);

    } 

    public static Command getAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }

}
