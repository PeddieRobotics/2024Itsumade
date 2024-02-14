package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Flywheel;

public class FlywheelTab extends ShuffleboardTabBase{
    
    private Flywheel flywheel = Flywheel.getInstance();
    private GenericEntry flywheelAtRPMEntry, flywheelCurrentEntry, flywheelDeltaEntry, flywheelMotorTempEntry, 
    flywheelRPMEntry, flywheelPercentOutputEntry, flywheelRightPercentOutputEntry, flywheelLeftPercentOutputEntry,
    flywheelVelocitySetpointEntry, flywheelVelocityRightSetpointEntry, flywheelVelocityLeftSetpointEntry, 
    flywheelToggleEntry, flywheelPIDToggleEntry,
    mkPEntry, mkIEntry, mkIzEntry, mkDEntry, mkFFEntry;

    public FlywheelTab(){}

    public void createEntries() {
    tab = Shuffleboard.getTab("FlywheelTab");

        try{
            flywheelAtRPMEntry = tab.add("Flywheel At RPM", false)
            .withSize(1, 1)
            .withPosition(0, 0)
            .getEntry();
            
            flywheelCurrentEntry = tab.add("Flywheel Current", 0.0)
            .withSize(1, 1)
            .withPosition(1, 0)
            .getEntry();

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1, 1)
            .withPosition(2, 0)
            .getEntry();

            flywheelMotorTempEntry = tab.add("Flywheel Motor Temperature", 0.0)
            .withSize(2, 1)
            .withPosition(3, 0)
            .getEntry();

            flywheelPercentOutputEntry = tab.add("Flywheel Both Percent Output", 0.0) 
            .withSize(2, 1)
            .withPosition(5, 0)
            .getEntry();
            
            flywheelRightPercentOutputEntry = tab.add("Flywheel Right Percent Output", 0.0) 
            .withSize(2, 1)
            .withPosition(7, 0)
            .getEntry();

            flywheelLeftPercentOutputEntry = tab.add("Flywheel Left Percent Output", 0.0) 
            .withSize(2, 1)
            .withPosition(9, 0)
            .getEntry();

            flywheelVelocitySetpointEntry = tab.add("Flywheel Both Velocity Setpoint", 0.0) 
            .withSize(2, 1)
            .withPosition(5, 2)
            .getEntry();

            flywheelVelocityLeftSetpointEntry = tab.add("Flywheel Left Velocity Setpoint", 0.0) 
            .withSize(2, 1)
            .withPosition(0, 1)
            .getEntry();

            flywheelVelocityRightSetpointEntry = tab.add("Flywheel Right Velocity Setpoint", 0.0) 
            .withSize(2, 1)
            .withPosition(2, 1)
            .getEntry();

            flywheelRPMEntry = tab.add("Flywheel Current RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(4, 1)
            .getEntry();

            flywheelToggleEntry = tab.add("Flywheel On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(1, 1)
            .withPosition(6, 1)
            .getEntry();

            flywheelPIDToggleEntry = tab.add("Flywheel PID On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(1, 1)
            .withPosition(7, 1)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) 
            .withSize(1, 1)
            .withPosition(8, 1)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0)
            .withSize(1, 1)
            .withPosition(9, 1)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0)
            .withSize(1, 1)
            .withPosition(10, 1)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0)
            .withSize(1, 1)
            .withPosition(0, 2)
            .getEntry();

            mkFFEntry = tab.add("kFF", 0.0) 
            .withSize(1, 1)
            .withPosition(1, 2)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update() {
        try{

            // if(flywheelToggleEntry.getBoolean(false)){
            //     flywheel.runFlywheelPercentOutput(flywheelPercentOutputEntry.getDouble(0.0));
            //     flywheel.runRightFlywheelPercentOutput(flywheelRightPercentOutputEntry.getDouble(0.0));
            //     flywheel.runLeftFlywheelPercentOutput(flywheelLeftPercentOutputEntry.getDouble(0.0));

            //     flywheel.runFlywheelVelocitySetpoint(flywheelVelocitySetpointEntry.getDouble(0.0));
            //     flywheel.runRightFlywheelVelocitySetpoint(flywheelVelocityRightSetpointEntry.getDouble(0.0));
            //     flywheel.runLeftFlywheelVelocitySetpoint(flywheelVelocityLeftSetpointEntry.getDouble(0.0));

            // }
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
