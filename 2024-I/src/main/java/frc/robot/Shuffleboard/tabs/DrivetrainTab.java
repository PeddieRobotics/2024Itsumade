package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Drivetrain;

public class DrivetrainTab extends ShuffleboardTabBase{

    private Drivetrain drivetrain = Drivetrain.getInstance();
    private GenericEntry mHeadingEntry, mkSEntry, mkVEntry, mkAEntry, 
    mkGEntry, mkFFEntry, mkDEntry, mkIEntry, mkIzEntry, mkPEntry, 
    mModuleRotations1Entry, mModuleRotations2Entry, mModuleRotations3Entry, mModuleRotations4Entry,
    mOdometryXEntry, mOdometryYEntry, mOdometryThetaEntry, 
    PIDToggleEntry;

    public DrivetrainTab(){}

    public void createEntries() {
        tab = Shuffleboard.getTab("DrivetrainTab");

        try{
            mHeadingEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(1, 1)
            .withPosition(0, 0)
            .getEntry();

            mkSEntry = tab.add("kS", 0.0) // DrivetrainConstants.kS
            .withSize(1, 1)
            .withPosition(1, 0)
            .getEntry();

            mkVEntry = tab.add("kV", 0.0) // DrivetrainConstants.kV
            .withSize(1, 1)
            .withPosition(2, 0)
            .getEntry();

            mkAEntry = tab.add("kFF", 0.0) // DrivetrainConstants.kA
            .withSize(1, 1)
            .withPosition(3, 0)
            .getEntry();

            mkFFEntry = tab.add("kP", 0.0) // DrivetrainConstants.kFF
            .withSize(1, 1)
            .withPosition(4, 0)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0) // DrivetrainConstants.kD
            .withSize(1, 1)
            .withPosition(5, 0)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0) // DrivetrainConstants.kI
            .withSize(1, 1)
            .withPosition(6, 0)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0) // DrivetrainConstants.kIz
            .withSize(1, 1)
            .withPosition(7, 0)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) // DrivetrainConstants.kP
            .withSize(1, 1)
            .withPosition(0, 1)
            .getEntry();

            mModuleRotations1Entry = tab.add("Module Rotations 1", 0.0) //drivetrain.getModuleRotations()[0]
            .withSize(1, 1)
            .withPosition(1, 1)
            .getEntry();

            mModuleRotations2Entry = tab.add("Module Rotations 2", 0.0) //drivetrain.getModuleRotations()[1]
            .withSize(1, 1)
            .withPosition(2, 1)
            .getEntry();

            mModuleRotations3Entry = tab.add("Module Rotations 3", 0.0) //drivetrain.getModuleRotations()[2]
            .withSize(1, 1)
            .withPosition(3, 1)
            .getEntry();

            mModuleRotations4Entry = tab.add("Module Rotations 4", 0.0) //drivetrain.getModuleRotations()[3]
            .withSize(1, 1)
            .withPosition(4, 1)
            .getEntry();

            mOdometryXEntry = tab.add("Odometry X", 0.0) //drivetrain.getPose().getX()
            .withSize(1, 1)
            .withPosition(5, 1)
            .getEntry();
    
            mOdometryYEntry = tab.add("Odometry Y", 0.0) //drivetrain.getPose().getY()
            .withSize(1, 1)
            .withPosition(6, 1)
            .getEntry();
    
            mOdometryThetaEntry = tab.add("Odometry Theta", 0.0) //drivetrain.getPose().getDegrees()
            .withSize(1, 1)
            .withPosition(7, 1)
            .getEntry();
            
            PIDToggleEntry = tab.add("PID Toggle", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 1)
            .withPosition(0, 2)
            .getEntry();
        } catch (IllegalArgumentException e) {}
    }

    @Override
    public void update(){
        try{
        } catch (IllegalArgumentException e) {}
    }
}
