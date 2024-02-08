package frc.robot.Shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.tabs.*;

public class ShuffleboardMain {
    private static ShuffleboardMain shuffleboardMain;

    private ArrayList<ShuffleboardTabBase> tabs = new ArrayList<ShuffleboardTabBase>();

    private AngleOverridesTab angleOverridesTab;
    private ArmTab armTab;
    private DrivetrainTab drivetrainTab;
    private FlywheelTab flywheelTab;
    private IntakeTab intakeTab;
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
        angleOverridesTab = new AngleOverridesTab();
        armTab = new ArmTab();
        drivetrainTab = new DrivetrainTab();
        flywheelTab = new FlywheelTab();
        intakeTab = new IntakeTab();
        limelightTab = new LimelightTab();
        operatorTab = new OperatorTab();

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

    public void setUpAutoSelector() {
        operatorTab.setUpAutoSelector();
    }
}
