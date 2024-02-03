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
    mOdometryXEntry, mOdometryYEntry, mOdometryThetaEntry, mPIDToggleEntry, 
    mUseHeadingCorrectionToggleEntry, mAllowDrivingToggleEntry;

    public void createEntries() {
        tab = Shuffleboard.getTab("DrivetrainTab");

        try{
            mHeadingEntry = tab.add("Heading", 0.0) //drivetrain.getHeading()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkSEntry = tab.add("kS", 0.0) // DrivetrainConstants.kS
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkVEntry = tab.add("kV", 0.0) // DrivetrainConstants.kV
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkAEntry = tab.add("kFF", 0.0) // DrivetrainConstants.kA
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkFFEntry = tab.add("kP", 0.0) // DrivetrainConstants.kFF
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0) // DrivetrainConstants.kD
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0) // DrivetrainConstants.kI
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0) // DrivetrainConstants.kIz
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) // DrivetrainConstants.kP
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mModuleRotations1Entry = tab.add("Module Rotations 1", 0.0) //drivetrain.getModuleRotations()[0]
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mModuleRotations2Entry = tab.add("Module Rotations 2", 0.0) //drivetrain.getModuleRotations()[1]
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mModuleRotations3Entry = tab.add("Module Rotations 3", 0.0) //drivetrain.getModuleRotations()[2]
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mModuleRotations4Entry = tab.add("Module Rotations 4", 0.0) //drivetrain.getModuleRotations()[3]
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mOdometryXEntry = tab.add("Odometry X", 0.0) //drivetrain.getPose().getX()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
    
            mOdometryYEntry = tab.add("Odometry Y", 0.0) //drivetrain.getPose().getY()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
    
            mOdometryThetaEntry = tab.add("Odometry Theta", 0.0) //drivetrain.getPose().getDegrees()
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mUseHeadingCorrectionToggleEntry = tab.add("Use Heading Correction", true) 
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mAllowDrivingToggleEntry = tab.add("Allow Driving", true)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mPIDToggleEntry = tab.add("PID Toggle", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
        } catch (IllegalArgumentException e) {}
    }

    public void update(){
        try{
            /*
                mOdometryX.setDouble(drivetrain.getPose().getX());
                mOdometryY.setDouble(drivetrain.getPose().getY());
                mOdometryTheta.setDouble(drivetrain.getPose().getRotation().getDegrees());
                mHeading.setDouble(drivetrain.getHeading());
                mModuleRotations1.setDouble(drivetrain.getModuleRotations()[0]);
                mModuleRotations2.setDouble(drivetrain.getModuleRotations()[1]);
                mModuleRotations3.setDouble(drivetrain.getModuleRotations()[2]);
                mModuleRotations4.setDouble(drivetrain.getModuleRotations()[3]);

                drivetrain.setUseHeadingCorrection(mUseHeadingCorrection.getBoolean(true));
                drivetrain.setAllowDriving(mAllowDriving.getBoolean(true));
             * Adjustable Drivetrain PID below, Do this later
             */
        } catch (IllegalArgumentException e) {}
    }
}
