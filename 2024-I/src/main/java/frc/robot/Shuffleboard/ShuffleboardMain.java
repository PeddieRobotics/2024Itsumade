package frc.robot.Shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shuffleboard.tabs.*;

public class ShuffleboardMain {
    private static ShuffleboardMain shuffleboardMain;

    private ArrayList<ShuffleboardTabBase> tabs = new ArrayList<ShuffleboardTabBase>();

    private AngleOverridesTab angleOverridesTab;
    private ArmTab armTab;
    // private DebugTab debugTab;
    private LiveTuningTab liveTuningTab;
    private DrivetrainTab drivetrainTab;
    private FlywheelTab flywheelTab;
    private IntakeHopperTab intakeHopperTab;
    private LimelightTab limelightTab;
    private OperatorTab operatorTab;

    public ShuffleboardMain(){
    }

    public static ShuffleboardMain getInstance() {
        if(shuffleboardMain == null){
            shuffleboardMain = new ShuffleboardMain();
        }
        return shuffleboardMain;
    }

    public void setUpTabs(){
        // angleOverridesTab = new AngleOverridesTab();
        // tabs.add(angleOverridesTab);
        // armTab = new ArmTab();
        // tabs.add(armTab);
        // drivetrainTab = new DrivetrainTab();
        // tabs.add(drivetrainTab);
        // debugTab = new DebugTab();
        // tabs.add(debugTab);
        // flywheelTab = new FlywheelTab();
        // tabs.add(flywheelTab);
        // intakeHopperTab = new IntakeHopperTab();
        // tabs.add(intakeHopperTab);
        // limelightTab = new LimelightTab();
        // tabs.add(limelightTab);
        operatorTab = new OperatorTab();
        // liveTuningTab = new LiveTuningTab();
        tabs.add(operatorTab);

        for (ShuffleboardTabBase tab : tabs){
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : tabs){
            tab.update();
        }
    }

    public Command getAutonomousCommand(){
        return operatorTab.getAutonomousCommand();
    }

    public double getGyroOffsetForTeleop(){
        return operatorTab.getGyroOffsetForTeleop() * (DriverStation.getAlliance().get()==Alliance.Red? 1 : -1);
    }

    public void setUpAutoSelectors() {
        try{
            operatorTab.configureAutoSelector();
            operatorTab.configureAutoSetupSelector();
        } catch(NullPointerException e){}
    }
}
;