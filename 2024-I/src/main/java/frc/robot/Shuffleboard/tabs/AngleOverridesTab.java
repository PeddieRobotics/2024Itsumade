package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Arm;

public class AngleOverridesTab extends ShuffleboardTabBase{ //Tentatively Complete, pay attention if prep & scoring positions are different later - Tony
    private Arm arm = Arm.getInstance();

    private GenericEntry ampPrepEntry, armAngleEntry, armPrepStateEntry, 
    armStowEntry, layupPrepEntry, LLPrepEntry;

    public AngleOverridesTab(){}

    public void createEntries() {
        tab = Shuffleboard.getTab("AngleOverridesTab");

        try{ //All of these sizes and positions are arbitrary, please consider changing later...
            ampPrepEntry = tab.add("Amp Prep", 0.0) //ArmConstants.kAmpPrepAngle
            .withSize(2,2)
            .withPosition(1,2)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0) //arm.getCurrentAngle()
            .withSize(2,2)
            .withPosition(4,5)
            .getEntry();
            
            armPrepStateEntry = tab.add("Arm Set Angle", 0.0) //arm.getCurrentPrepState()
            .withSize(2,2)
            .withPosition(7,8)
            .getEntry();
            
            armStowEntry = tab.add("Arm Stow", 0.0) //ArmConstants.kStowAngle
            .withSize(2,2)
            .withPosition(10,11)
            .getEntry();
            
            layupPrepEntry = tab.add("Layup Prep", 0.0) //ArmConstants.kLayupAngle
            .withSize(2,2)
            .withPosition(13,14)
            .getEntry();
            
            LLPrepEntry = tab.add("LL Prep", 0.0) //ArmConstants.kLLAngle
            .withSize(2,2)
            .withPosition(16,17)
            .getEntry();            
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            /*
            arm.setkAmpAngle(ampPrepEntry.getDouble(ArmConstants.kAmpPrepAngle));
            armAngleEntry.setDouble(arm.getCurrentAngle());
            armPrepStateEntry.setString(arm.getCurrentPrepState());
            arm.setkStowAngle(armStowEntry.getDouble(ArmConstants.kStowAngle));
            arm.setkLayupAngle(layupPrepEntry.getDouble(ArmConstants.kLayupAngle));
            arm.setkLLAngle(LLPrepStateEntry.getDouble(ArmConstants.kLLAngle));
            */
        } catch(IllegalArgumentException e){}
    }
}