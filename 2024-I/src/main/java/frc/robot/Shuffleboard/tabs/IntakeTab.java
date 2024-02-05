package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Intake;

public class IntakeTab extends ShuffleboardTabBase{

    private Intake intake = Intake.getInstance();

    private GenericEntry intakeToggleEntry , intakeCurrentSpeedEntry, intakeMotorTempEntry, intakeSetSpeedEntry, intakeHasGamepieceEntry;

    public IntakeTab(){}

    public void createEntries() {
        tab = Shuffleboard.getTab("IntakeTab");
        
        try{
            intakeToggleEntry = tab.add("Intake On", false) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            intakeCurrentSpeedEntry = tab.add("Intake Current Speed", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            intakeMotorTempEntry = tab.add("Intake Motor Temperature", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            intakeSetSpeedEntry = tab.add("Intake Set Speed", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            intakeHasGamepieceEntry = tab.add("Intake Has Gamepiece", false) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update(){
        try{
            /*
             * intakeCurrentSpeedEntry.setDouble(intake.getSpeed());
             * intakeHasGamepieceEntry.setBoolean(intake.hasGamepiece());
             * intakeMotorTempEntry.setDouble(intake.getMotorTemperature());
             * 
             * if(intakeToggleEntry.getBoolean()){
             *      intake.setSpeed(intakeSetSpeedEntry.getDouble())
             * }
             */
        } catch (IllegalArgumentException e){}
    }
}
