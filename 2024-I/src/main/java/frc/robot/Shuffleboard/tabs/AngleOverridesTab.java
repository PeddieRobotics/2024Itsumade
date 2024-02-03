package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;

public class AngleOverridesTab extends ShuffleBoardTabBase{
    private Arm arm = Arm.getInstance();

    private GenericEntry armAngleEntry, armForceStowEntry, armPrepStateEntry, armSetEntry, 
    armStowEntry, ampPrepEntry, layupPrepEntry, LLPrepStateEntry;

    public void createEntries() {
        tab = Shuffleboard.getTab("AngleOverridesTab");

        try{ //All of these sizes and positions are arbitrary, please consider changing later...
            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(2,2)
            .withPosition(17,13)
            .getEntry();

            armTempEntry = tab.add("Arm Temp", 0.0)
            .withSize(2,2)
            .withPosition(20,20)
            .getEntry();            

            cameraWidget = tab.addCamera("Camera", "CameraName", "url") 
            .withSize(2,2)
            .withPosition(1,7);
        
            current1Entry = tab.add("Current Channel 1", 0.0)
            .withSize(2,2)
            .withPosition(1,7)
            .getEntry();

            current2Entry = tab.add("Current Channel 2", 0.0)
            .withSize(2,2)
            .withPosition(4,10)
            .getEntry();

            current3Entry = tab.add("Current Channel 3", 0.0)
            .withSize(2,2)
            .withPosition(7, 13)
            .getEntry();

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(2,2)
            .withPosition(13, 10)
            .getEntry();

            flywheelTempEntry = tab.add("Flywheel Temp", 0.0)
            .withSize(2,2)
            .withPosition(25, 11)
            .getEntry();            
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            // armAngleEntry.setDouble(Arm.getArmAngle());
            //armTempEntry.setDouble(Arm.getArmTemperature());
            current1Entry.setDouble(pdh.getCurrent(1));
            current2Entry.setDouble(pdh.getCurrent(2));
            current3Entry.setDouble(pdh.getCurrent(3));
            //flywheelDeltaEntry.setDouble(flywheel.getRPM());
            //flywheelTempEntry.setDouble(flywheel.getMotorTemperature());
        } catch(IllegalArgumentException e){}
    }
}
