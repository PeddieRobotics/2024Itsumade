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
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0) //arm.getCurrentAngle()
            .withSize(1,1)
            .withPosition(1,0)
            .getEntry();
            
            armPrepStateEntry = tab.add("Arm Set Angle", 0.0) //arm.getCurrentPrepState()
            .withSize(1,1)
            .withPosition(2,0)
            .getEntry();
            
            armStowEntry = tab.add("Arm Stow", 0.0) //ArmConstants.kStowAngle
            .withSize(1,1)
            .withPosition(3,0)
            .getEntry();
            
            layupPrepEntry = tab.add("Layup Prep", 0.0) //ArmConstants.kLayupAngle
            .withSize(1,1)
            .withPosition(4,0)
            .getEntry();
            
            LLPrepEntry = tab.add("LL Prep", 0.0) //ArmConstants.kLLAngle
            .withSize(1,1)
            .withPosition(5,0)
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