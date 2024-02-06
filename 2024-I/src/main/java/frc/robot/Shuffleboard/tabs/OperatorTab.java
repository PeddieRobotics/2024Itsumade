package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class OperatorTab extends ShuffleboardTabBase{
    private PowerDistribution pdh = new PowerDistribution (1, ModuleType.kRev);
    private Arm arm = Arm.getInstance();
    private Autonomous autonomous = Autonomous.getInstance();
    private Climber climber = Climber.getInstance();
    private Flywheel flywheel = Flywheel.getInstance();
    private Intake intake = Intake.getInstance();
    private Limelight limelight = Limelight.getInstance();

    private ComplexWidget autoChooser, cameraWidget;

    private GenericEntry armAngleEntry, armTempEntry, current1Entry, current2Entry, 
    current3Entry, flywheelAtRPMEntry, flywheelSetRPMEntry, flywheelDeltaEntry, 
    flywheelTempEntry;

    //Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        autoRoutineSelector = new SendableChooser<Command>();

        try{ 
            state = tab.add("State", "STOW")
            .withSize(1,2)
            .withPosition(2,5)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,2)
            .withPosition(4,6)
            .getEntry();

            armTempEntry = tab.add("Arm Temp", 0.0)
            .withSize(1,2)
            .withPosition(4,8)
            .getEntry();            

            cameraWidget = tab.addCamera("Camera", "CameraName", "url") 
            .withSize(5,5)
            .withPosition(1,1);
        
            current1Entry = tab.add("Current Channel 1", 0.0)
            .withSize(1,1)
            .withPosition(3,7)
            .getEntry();

            current2Entry = tab.add("Current Channel 2", 0.0)
            .withSize(1,1)
            .withPosition(3,8)
            .getEntry();

            current3Entry = tab.add("Current Channel 3", 0.0)
            .withSize(1,1)
            .withPosition(3, 9)
            .getEntry();

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1,2)
            .withPosition(1, 6)
            .getEntry();

            flywheelSetRPMEntry = tab.add("Flywheel Set RPM", 0.0)
            .withSize(1,2)
            .withPosition(1, 8)
            .getEntry();            

            flywheelTempEntry = tab.add("Flywheel Temp", 0.0)
            .withSize(1,2)
            .withPosition(2, 7)
            .getEntry();            
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            // arm.setArmAngle(armAngleEntry.getDouble(ArmConstants.kArmAngle));
            //armTempEntry.setDouble(Arm.getArmTemperature());
            current1Entry.setDouble(pdh.getCurrent(1));
            current2Entry.setDouble(pdh.getCurrent(2));
            current3Entry.setDouble(pdh.getCurrent(3));
            //flywheelAtRPMEntry.setBoolean(FlywheelConstants.kAtRPM)
            //flywheel.setFlywheelRPM(flywheelSetRPMEntry.getDouble(FlywheelConstants.kShootingRPM)); 
            //^^Purely arbitrary, not sure if we will need a lookup table with additional rpm values so this is just here for now
            //flywheelTempEntry.setDouble(flywheel.getMotorTemperature());
        } catch(IllegalArgumentException e){}
    }

    public void setUpAutoSelector() {
        // TODO setting up the auto selector
        Hashtable<String,Command> autoRoutines = autonomous.getAutoRoutines();
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }
        autoChooser = tab.add("Auto routine", autoRoutineSelector).withSize(5,2).withPosition(16,1); // comp settings: withPosition(16,1);

    }

    public Command getAutonomousCommand() {
        // TODO getting the selected autonomous routine
        return autoRoutineSelector.getSelected();
    }
}
