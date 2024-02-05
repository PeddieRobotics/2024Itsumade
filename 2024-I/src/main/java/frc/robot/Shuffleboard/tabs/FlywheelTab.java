package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Flywheel;

public class FlywheelTab extends ShuffleboardTabBase{
    
    private Flywheel flywheel = Flywheel.getInstance();
    private GenericEntry flywheelAtRPMEntry, flywheelCurrentEntry, flywheelDeltaEntry, flywheelMotorTempEntry, 
    flywheelRPMEntry, flywheelSetpointEntry, flywheelToggleEntry, flywheelPIDToggleEntry,
    mkPEntry, mkIEntry, mkIzEntry, mkDEntry, mkFFEntry;

    public void createEntries() {
        try{
            flywheelAtRPMEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
            
            flywheelCurrentEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelDeltaEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelMotorTempEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelSetpointEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelRPMEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelToggleEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mHeadingEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkPEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIzEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkDEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkFFEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    public void update() {
        try{

        }  catch (IllegalArgumentException e){}
    }
}
