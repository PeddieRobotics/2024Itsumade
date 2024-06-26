package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class IntakeHopperTab extends ShuffleboardTabBase{

    // private Intake intake = Intake.getInstance();
    // private Hopper hopper = Hopper.getInstance();

    private GenericEntry hopperSetSpeedEntry, hopperToggleEntry, 
    intakeToggleEntry , intakeCurrentSpeedEntry, intakeMotorTempEntry, 
    intakeSetSpeedEntry, intakeHasGamepieceEntry;

    public IntakeHopperTab(){}

    public void createEntries() {
        tab = Shuffleboard.getTab("IntakeHopperTab");
        
        try{
            hopperSetSpeedEntry = tab.add("Hopper Set Speed", 0.0)
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            hopperToggleEntry = tab.add("Hopper On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(1, 1)
            .withPosition(1, 0)
            .getEntry();

            intakeToggleEntry = tab.add("Intake On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(1, 1)
            .withPosition(2, 0)
            .getEntry();

            intakeCurrentSpeedEntry = tab.add("Intake Current Speed", 0.0) 
            .withSize(1, 1)
            .withPosition(3, 0)
            .getEntry();

            intakeMotorTempEntry = tab.add("Intake Motor Temperature", 0.0)
            .withSize(1, 1)
            .withPosition(4, 0)
            .getEntry();

            intakeSetSpeedEntry = tab.add("Intake Set Speed", 0.0) 
            .withSize(1, 1)
            .withPosition(5, 0)
            .getEntry();

            intakeHasGamepieceEntry = tab.add("Intake Has Gamepiece", false) 
            .withSize(1, 1)
            .withPosition(6, 0)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update(){
        try{
        } catch (IllegalArgumentException e){}
    }
}
