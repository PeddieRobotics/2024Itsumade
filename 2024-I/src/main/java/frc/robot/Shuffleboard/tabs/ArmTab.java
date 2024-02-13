package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Arm;

public class ArmTab extends ShuffleboardTabBase {

    private Arm arm = Arm.getInstance();
    private GenericEntry armAngleEntry, armCurrentEntry, armSpeedEntry, armTempEntry,
    armVoltageEntry, mAuxilaryCLToggleEntry, mClosedLoopToggleEntry,
    mkAEntry, mkDEntry, mkFFEntry, mkGEntry,
    mkIEntry, mkIzEntry, mkPEntry, mkVEntry,
    mPIDToggleEntry, mArbitraryFFEntry;
    /*
     * NOTE: TRIED TO STUFF USEFUL FOR MOTION MAGIC, BUT NOT SURE
     * IF I ADDED EVERYTHING - ADD/DELETE WHATEVER ELSE IS NECESSARY
     * - Tony
     */

     public ArmTab(){}

    public void createEntries() {
        tab = Shuffleboard.getTab("ArmTab");

        try { // All of these sizes and positions are arbitrary, please consider changing later...
            armAngleEntry = tab.add("Arm Angle", 0.0) // arm.getCurrentAngle()
            .withSize(1, 1)
            .withPosition(0, 0)
            .getEntry();

            armCurrentEntry = tab.add("Arm Current", 0.0) // arm.getArmCurrent
            .withSize(1, 1)
            .withPosition(1, 0)
            .getEntry();
        
            armSpeedEntry = tab.add("Arm Percent Output", 0.0) // ArmConstants.kSpeed, setArmPercentOutput()
            .withSize(2, 1)
            .withPosition(2, 0)
            .getEntry();

            armTempEntry = tab.add("Arm Motor Temperature", 0.0) // arm.getCurrentMotorTemperature()
            .withSize(3, 1)
            .withPosition(4, 0)
            .getEntry();

            armVoltageEntry = tab.add("Arm Voltage", 0.0) // arm.getCurrentVoltage()
            .withSize(1, 1)
            .withPosition(7, 0)
            .getEntry();

            mAuxilaryCLToggleEntry = tab.add("Auxilary Closed Loop [2] Toggle", false) 
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 1)
            .withPosition(8, 0)
            .getEntry();

            mClosedLoopToggleEntry = tab.add("Closed Loop [1] Toggle", false) 
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 1)
            .withPosition(10, 0)
            .getEntry();

            mkAEntry = tab.add("kA", 0.0) // ArmConstants.kA
            .withSize(1, 1)
            .withPosition(0, 1)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0) // ArmConstants.kD
            .withSize(1, 1)
            .withPosition(1, 1)
            .getEntry();

            mkFFEntry = tab.add("kFF", 0.0) // ArmConstants.kFF
            .withSize(1, 1)
            .withPosition(2, 1)
            .getEntry();

            mkGEntry = tab.add("kG", 0.0) // ArmConstants.kG
            .withSize(1, 1)
            .withPosition(3, 1)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0) // ArmConstants.kI
            .withSize(1, 1)
            .withPosition(4, 1)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0) // ArmConstants.kIz
            .withSize(1, 1)
            .withPosition(5, 1)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) // ArmConstants.kP
            .withSize(1, 1)
            .withPosition(6, 1)
            .getEntry();

            mkVEntry = tab.add("kV", 0.0) // ArmConstants.kV
            .withSize(1, 1)
            .withPosition(7, 1)
            .getEntry();

            mPIDToggleEntry = tab.add("PID Toggle", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(0, 2)
            .getEntry();

            mArbitraryFFEntry = tab.add("Arbitrary FF", 0.0)
            .withSize(1, 1)
            .withPosition(1, 2)
            .getEntry();
        } catch (IllegalArgumentException e) {}
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            /*
            armAngleEntry.setDouble(arm.getCurrentAngle());
            armCurrentEntry.setDouble(arm.getArmCurrent());
            arm.setArmPercentOutput(armSpeedEntry.getDouble(ArmConstants.kSpeed));
            armTempEntry.setDouble(arm.getCurrentMotorTemperature());
            armVoltageEntry.setDouble(arm.getCurrentVoltage());

            Motion magic with PID toggle implementation should go below, will add later... DO THIS LATER, copied in the flywheel pid toggle tentative logic that I didn't double check
             if(flywheelToggleEntry.getBoolean()){
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
        } catch(IllegalArgumentException e){}
    }
}