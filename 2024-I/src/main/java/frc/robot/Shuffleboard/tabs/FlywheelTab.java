package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Flywheel;

public class FlywheelTab extends ShuffleboardTabBase{
    
    private Flywheel flywheel = Flywheel.getInstance();
    private GenericEntry flywheelAtRPMEntry, flywheelCurrentEntry, flywheelDeltaEntry, flywheelMotorTempEntry, 
    flywheelRPMEntry, flywheelSetpointEntry, flywheelToggleEntry, flywheelPIDToggleEntry,
    mkPEntry, mkIEntry, mkIzEntry, mkDEntry, mkFFEntry;

    public FlywheelTab(){}

    public void createEntries() {
    tab = Shuffleboard.getTab("FlywheelTab");

        try{
            flywheelAtRPMEntry = tab.add("Flywheel At RPM", false)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
            
            flywheelCurrentEntry = tab.add("Flywheel Current", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelMotorTempEntry = tab.add("Flywheel Motor Temperature", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelSetpointEntry = tab.add("Flywheel Setpoint", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelRPMEntry = tab.add("Flywheel Current RPM", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelToggleEntry = tab.add("Flywheel On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            flywheelPIDToggleEntry = tab.add("Flywheel PID On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkFFEntry = tab.add("kFF", 0.0) 
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update() {
        try{
            /* 
             * flywheelAtRPMEntry.setBoolean((flywheel.AtRPM());
             * flywheelCurrentEntry.setDouble((flywheel.getCurrent());
             * flywheel.setFlywheelDelta(flywheelDeltaEntry.getDouble());
             * flywheelMotorTempEntry.setDouble((flywheel.getMotorTemperature());
             * flywheelRPMEntry().setDouble(flywheel.getRPM());
             * 
             * if(flywheelToggleEntry.getBoolean()){
             *      flywheel.setFlywheelSetpoint(flywheelSetpointEntry.getDouble())
             *      if(flywheelPIDToggleEntry.getBoolean()){
             *          flywheel.updatePIDController(mkPEntry.getDouble(),
             *          mkIEntry.getDouble(), mkDEntry.getDouble(),
             *          mkIzEntry.getDouble(), mkFFEntry.getDouble(), 0);
             *      } else if(flywheelPIDToggleEntry.getBoolean(false)){
             *      flywheel.updatePIDController(FlywheelConstants.kP, FlywheelConstants.kI,
             *      FlywheelConstants.kD, FlywheelConstants.kIz, FlywheelConstants.kFF, 0)
             *      }
             * }
            */
        }  catch (IllegalArgumentException e){}
    }
}
