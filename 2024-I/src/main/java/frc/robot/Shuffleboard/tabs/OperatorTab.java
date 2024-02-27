package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;

public class OperatorTab extends ShuffleboardTabBase{
    private ComplexWidget autoChooser, cameraWidget;

    private Arm arm;
    private Autonomous autonomous;
    private Climber climber;
    private Flywheel flywheel;
    private Hopper hopper;
    private Intake intake;
    private LimelightShooter limelightShooter;
    private LimelightIntake limelightIntake;

    private GenericEntry stateEntry, armSetpointEntry, armAngleEntry, armTempEntry, 
    flywheelAtRPMEntry, flywheelSetRPMEntry, flywheelDeltaEntry, 
    flywheelTempEntry, flywheelVelocitySetpointEntry, flywheelVelocityRightSetpointEntry, 
    flywheelLeftRPMEntry, flywheelRightRPMEntry, 
    flywheelVelocityLeftSetpointEntry, flywheelToggleEntry, isIndexedOverrideEntry, 
    isGamePieceIndexedEntry, stowAfterShootOverrideEntry, topSensorEntry, bottomSensorEntry;

    //Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;

    public OperatorTab(){
        arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        climber = Climber.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        limelightIntake = LimelightIntake.getInstance();
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        autoRoutineSelector = new SendableChooser<Command>();

        try{ 
            stateEntry = tab.add("State", "STOW")
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            armSetpointEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,1)
            .withPosition(1,0)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,1)
            .withPosition(5,7)
            .getEntry();

            armTempEntry = tab.add("Arm Temp", 0.0)
            .withSize(1,1)
            .withPosition(2,0)
            .getEntry();            

            cameraWidget = tab.addCamera("Camera", "CameraName", "url") //LATER, with one of the limelights
            .withSize(5,5)
            .withPosition(4,3);

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1,1)
            .withPosition(6, 0)
            .getEntry();

            flywheelSetRPMEntry = tab.add("Flywheel Set RPM", 0.0)
            .withSize(1,1)
            .withPosition(7, 0)
            .getEntry();            

            flywheelTempEntry = tab.add("Flywheel Temp", 0.0)
            .withSize(1,1)
            .withPosition(0, 1)
            .getEntry();
            
            flywheelVelocitySetpointEntry = tab.add("Flywheel Both Velocity Setpoint", 0.0) //Not sure this is needed anymore, someone validate - Tony
            .withSize(1, 1)
            .withPosition(1, 1)
            .getEntry();

            flywheelVelocityLeftSetpointEntry = tab.add("Flywheel Left Velocity Setpoint", 0.0) 
            .withSize(1, 1)
            .withPosition(2, 1)
            .getEntry();

            flywheelVelocityRightSetpointEntry = tab.add("Flywheel Right Velocity Setpoint", 0.0) 
            .withSize(1, 1)
            .withPosition(3, 1)
            .getEntry();

            flywheelLeftRPMEntry = tab.add("Flywheel Left RPM", 0.0) 
            .withSize(1, 1)
            .withPosition(5, 7)
            .getEntry();

            flywheelRightRPMEntry = tab.add("Flywheel Right RPM", 0.0) 
            .withSize(1, 1)
            .withPosition(5, 8)
            .getEntry();

            flywheelToggleEntry = tab.add("Flywheel On", false)
            .withWidget(BuiltInWidgets.kToggleButton) 
            .withSize(1, 1)
            .withPosition(4, 1)
            .getEntry();

            isIndexedOverrideEntry = tab.add("Piece Indexed Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1,1)
            .withPosition(5, 1)
            .getEntry();

            isGamePieceIndexedEntry = tab.add("INDEXED?", hopper.isGamepieceIndexed())
            .withSize(1,1)
            .withPosition(5, 2)
            .getEntry();

            stowAfterShootOverrideEntry = tab.add("Stow After Shoot Override", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1,1)
            .withPosition(5, 3)
            .getEntry();

            topSensorEntry = tab.add("Top Sensor?", false)
            .withSize(2,1)
            .withPosition(5, 4)
            .getEntry();

            bottomSensorEntry = tab.add("Bottom Sensor?", false)
            .withSize(2,1)
            .withPosition(6, 0)
            .getEntry();


        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {

            if(flywheelToggleEntry.getBoolean(false)){
                flywheel.runRightFlywheelVelocitySetpoint(flywheelVelocityRightSetpointEntry.getDouble(0.0));
                flywheel.runLeftFlywheelVelocitySetpoint(flywheelVelocityLeftSetpointEntry.getDouble(0.0));
            }

            arm.setArmAngle(armSetpointEntry.getDouble(0.0));
            armAngleEntry.getDouble(arm.getArmAngleDegrees());
            armTempEntry.getDouble(arm.getArmTemperature());

            flywheelAtRPMEntry.getBoolean(false);
            flywheelLeftRPMEntry.setDouble(flywheel.getFlywheelLeftRPM());
            flywheelRightRPMEntry.setDouble(flywheel.getFlywheelRightRPM());

            flywheel.runLeftFlywheelVelocitySetpoint(flywheelVelocityLeftSetpointEntry.getDouble(0.0));
            flywheel.runRightFlywheelVelocitySetpoint(flywheelVelocityRightSetpointEntry.getDouble(0.0));

            flywheel.setRPMDelta(flywheelDeltaEntry.getDouble(0));
            stowAfterShootOverrideEntry.getBoolean(true);
            isIndexedOverrideEntry.getBoolean(false);
            isGamePieceIndexedEntry.getBoolean(false);
            stateEntry.setString(arm.getState());

            topSensorEntry.setBoolean(hopper.getTopSensor());
            bottomSensorEntry.setBoolean(hopper.getBottomSensor());
        } catch(IllegalArgumentException e){}
    }

    public void setUpAutoSelector() {
        //TODO setting up the auto selector
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
