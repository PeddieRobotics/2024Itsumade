package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

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
    private ComplexWidget autoChooser, cameraWidget;

    private Arm arm;
    private Autonomous autonomous;
    private Flywheel flywheel;
    private Hopper hopper;
    private Intake intake;
    private Superstructure superstructure;

    private GenericEntry stateEntry, armAngleEntry,
    flywheelAtRPMEntry, flywheelDeltaEntry, flywheelLeftRPMEntry, 
    flywheelRightRPMEntry, isIndexedOverrideEntry, 
    isGamePieceIndexedEntry, stowAfterShootOverrideEntry, topSensorEntry, bottomSensorEntry,
    hasGamePieceEntry;

    //Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;

    public OperatorTab(){
        arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        superstructure = Superstructure.getInstance();
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        autoRoutineSelector = new SendableChooser<Command>();

        try{ 
            stateEntry = tab.add("State", "STOW")
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,1)
            .withPosition(0,1)
            .getEntry();

            // cameraWidget = tab.addCamera("Camera", "CameraName", "url") //LATER, with one of the limelights
            // .withSize(5,5)
            // .withPosition(4,3);

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1,1)
            .withPosition(0, 2)
            .getEntry();

            flywheelLeftRPMEntry = tab.add("Flywheel Left RPM", 0.0) 
            .withSize(1, 1)
            .withPosition(0, 3)
            .getEntry();

            flywheelRightRPMEntry = tab.add("Flywheel Right RPM", 0.0) 
            .withSize(1, 1)
            .withPosition(0, 4)
            .getEntry();

            isIndexedOverrideEntry = tab.add("Piece Indexed Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1,1)
            .withPosition(0, 5)
            .getEntry();

            isGamePieceIndexedEntry = tab.add("INDEXED?", hopper.isGamepieceIndexed())
            .withSize(1,1)
            .withPosition(0, 6)
            .getEntry();

            stowAfterShootOverrideEntry = tab.add("Stow After Shoot Override", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1,1)
            .withPosition(0, 7)
            .getEntry();

            topSensorEntry = tab.add("Top Sensor?", false)
            .withSize(2,1)
            .withPosition(1, 0)
            .getEntry();

            bottomSensorEntry = tab.add("Bottom Sensor?", false)
            .withSize(2,1)
            .withPosition(1, 2)
            .getEntry();   

            hasGamePieceEntry = tab.add("Has Gamepiece?", false)
            .withSize(4,4)
            .withPosition(3, 3)
            .getEntry();            
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            armAngleEntry.setDouble(arm.getArmAngleDegrees());

            flywheelAtRPMEntry.getBoolean(false);
            flywheelLeftRPMEntry.setDouble(flywheel.getFlywheelLeftRPM());
            flywheelRightRPMEntry.setDouble(flywheel.getFlywheelRightRPM());
            flywheel.setRPMDelta(flywheelDeltaEntry.getDouble(0));

            stowAfterShootOverrideEntry.getBoolean(true);
            isIndexedOverrideEntry.getBoolean(false);
            isGamePieceIndexedEntry.getBoolean(false); //check this and the above later
            stateEntry.setString(superstructure.getRobotState());

            topSensorEntry.setBoolean(hopper.getTopSensor());
            bottomSensorEntry.setBoolean(hopper.getBottomSensor());

            hasGamePieceEntry.setBoolean(hopper.hasGamepiece() || intake.hasGamepiece());
        } catch(IllegalArgumentException e){}
    }

    public void setUpAutoSelector() {
        //TODO setting up the auto selector
        Hashtable<String,Command> autoRoutines = autonomous.getAutoRoutines();
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }
        autoChooser = tab.add("Auto routine", autoRoutineSelector).withSize(5,2).withPosition(7,7); 
    }

    public Command getAutonomousCommand() {
        // TODO getting the selected autonomous routine
        return autoRoutineSelector.getSelected();
    }
}
