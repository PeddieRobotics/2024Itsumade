package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

public class OperatorTab extends ShuffleboardTabBase{
    private SendableChooser<Command> autoChooser;
    private SendableChooser<String> autoSetupChooser;

    private ComplexWidget autoChooserWidget, autoSetupWidget, cameraWidget;

    private Arm arm;
    private Autonomous autonomous;
    private Flywheel flywheel;
    private Hopper hopper;
    private Intake intake;
    private Superstructure superstructure;

    private GenericEntry stateEntry, armAngleEntry,
    flywheelAtRPMEntry, armDeltaEntry, flywheelDeltaEntry, flywheelLeftRPMEntry, 
    flywheelRightRPMEntry, isIndexedOverrideEntry, topSensorEntry, bottomSensorEntry,
    hasGamePieceEntry;

    public OperatorTab(){
        arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        flywheel = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        superstructure = Superstructure.getInstance();
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        try{ 
            stateEntry = tab.add("State", "STOW")
            .withSize(1,1)
            .withPosition(0,0)
            .getEntry();

            armAngleEntry = tab.add("Arm Angle", 0.0)
            .withSize(1,1)
            .withPosition(0,1)
            .getEntry();

            cameraWidget = tab.addCamera("Camera", "LL Intake", "http://10.58.95.53:5800")
            .withSize(5,5)
            .withPosition(5,1);

            flywheelDeltaEntry = tab.add("Flywheel Delta", 0.0)
            .withSize(1,1)
            .withPosition(1, 2)
            .getEntry();

            armDeltaEntry = tab.add("Arm Delta", 0.0)
            .withSize(1,1)
            .withPosition(0, 2)
            .getEntry();

            flywheelLeftRPMEntry = tab.add("Flywheel Left RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 0)
            .getEntry();

            flywheelRightRPMEntry = tab.add("Flywheel Right RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 1)
            .getEntry();

            flywheelAtRPMEntry = tab.add("Flywheel At RPM", 0.0) 
            .withSize(2, 1)
            .withPosition(1, 3)
            .getEntry();

            // Return to this later
            // isIndexedOverrideEntry = tab.add("Piece Indexed Override", false)
            // .withWidget(BuiltInWidgets.kToggleButton)
            // .withSize(2,1)
            // .withPosition(3, 0)
            // .getEntry();

            topSensorEntry = tab.add("Top Sensor?", false)
            .withSize(1,1)
            .withPosition(2, 0)
            .getEntry();

            bottomSensorEntry = tab.add("Bottom Sensor?", false)
            .withSize(1,1)
            .withPosition(2, 1)
            .getEntry();   

            hasGamePieceEntry = tab.add("Has Gamepiece?", false)
            .withSize(2,2)
            .withPosition(0, 3)
            .getEntry();            
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() {
        try {
            armAngleEntry.setDouble(arm.getArmAngleDegrees());

            flywheelAtRPMEntry.getBoolean(flywheel.isAtRPM());
            flywheelLeftRPMEntry.setDouble(flywheel.getFlywheelLeftRPM());
            flywheelRightRPMEntry.setDouble(flywheel.getFlywheelRightRPM());
            flywheel.setRPMDelta(flywheelDeltaEntry.getDouble(0));

            arm.setArmDelta(armDeltaEntry.getDouble(0));

            // isIndexedOverrideEntry.getBoolean(false);  // Return to this later
            stateEntry.setString(superstructure.getRobotState());

            topSensorEntry.setBoolean(hopper.getTopSensor());
            bottomSensorEntry.setBoolean(hopper.getBottomSensor());

            hasGamePieceEntry.setBoolean(hopper.hasGamepiece());
        } catch(IllegalArgumentException e){}
    }

    public void configureAutoSelector(){
        autoChooser = autonomous.getAutoChooser();
        autoChooserWidget = tab.add("Auto routine", autoChooser).withSize(4,1).withPosition(0,4);
    }

    public void configureAutoSetupSelector(){
        autoSetupChooser = new SendableChooser<String>();
        autoSetupChooser.setDefaultOption("NONE/TELEOP", "NONE/TELEOP");
        autoSetupChooser.addOption("SOURCE", "SOURCE");
        autoSetupChooser.addOption("AMP", "AMP");
        autoSetupChooser.addOption("CENTER", "CENTER");
        autoSetupWidget = tab.add("Auto setup", autoSetupChooser).withSize(4,1).withPosition(0,5);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public double getGyroOffsetForTeleop(){
        if(autoSetupChooser.getSelected().equals("NONE/TELEOP")){
            return 0.0;
        }
        else if(autoSetupChooser.getSelected().equals("SOURCE")){
            return -120.0;
        }
        else if(autoSetupChooser.getSelected().equals("AMP")){
            return 120.0;
        }
        else if(autoSetupChooser.getSelected().equals("CENTER")){
            return 180.0;
        }       
        else{
            return 0.0;
        }
    }

}
