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
            .withPosition(1, 1)
            .getEntry();

            mkSEntry = tab.add("kS", 0.0) // DrivetrainConstants.kS
            .withSize(1, 1)
            .withPosition(1, 2)
            .getEntry();

            mkVEntry = tab.add("kV", 0.0) // DrivetrainConstants.kV
            .withSize(1, 1)
            .withPosition(1, 3)
            .getEntry();

            mkAEntry = tab.add("kFF", 0.0) // DrivetrainConstants.kA
            .withSize(1, 1)
            .withPosition(1, 4)
            .getEntry();

            mkFFEntry = tab.add("kP", 0.0) // DrivetrainConstants.kFF
            .withSize(1, 1)
            .withPosition(1, 5)
            .getEntry();

            mkDEntry = tab.add("kD", 0.0) // DrivetrainConstants.kD
            .withSize(1, 1)
            .withPosition(1, 6)
            .getEntry();

            mkIEntry = tab.add("kI", 0.0) // DrivetrainConstants.kI
            .withSize(1, 1)
            .withPosition(1, 7)
            .getEntry();

            mkIzEntry = tab.add("kIz", 0.0) // DrivetrainConstants.kIz
            .withSize(1, 1)
            .withPosition(1, 8)
            .getEntry();

            mkPEntry = tab.add("kP", 0.0) // DrivetrainConstants.kP
            .withSize(1, 1)
            .withPosition(1, 9)
            .getEntry();

            mModuleRotations1Entry = tab.add("Module Rotations 1", 0.0) //drivetrain.getModuleRotations()[0]
            .withSize(1, 1)
            .withPosition(2, 1)
            .getEntry();

            mModuleRotations2Entry = tab.add("Module Rotations 2", 0.0) //drivetrain.getModuleRotations()[1]
            .withSize(1, 1)
            .withPosition(2, 2)
            .getEntry();

            mModuleRotations3Entry = tab.add("Module Rotations 3", 0.0) //drivetrain.getModuleRotations()[2]
            .withSize(1, 1)
            .withPosition(2, 3)
            .getEntry();

            mModuleRotations4Entry = tab.add("Module Rotations 4", 0.0) //drivetrain.getModuleRotations()[3]
            .withSize(1, 1)
            .withPosition(2, 4)
            .getEntry();

            mOdometryXEntry = tab.add("Odometry X", 0.0) //drivetrain.getPose().getX()
            .withSize(1, 1)
            .withPosition(2, 5)
            .getEntry();
    
            mOdometryYEntry = tab.add("Odometry Y", 0.0) //drivetrain.getPose().getY()
            .withSize(1, 1)
            .withPosition(2, 6)
            .getEntry();
    
            mOdometryThetaEntry = tab.add("Odometry Theta", 0.0) //drivetrain.getPose().getDegrees()
            .withSize(1, 1)
            .withPosition(2, 7)
            .getEntry();
            
            PIDToggleEntry = tab.add("PID Toggle", true)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(1, 2)
            .withPosition(3, 5)
            .getEntry();
        } catch (IllegalArgumentException e) {}
    }

    @Override
    public void update(){
        try{
            mHeadingEntry.setDouble(drivetrain.getHeading());

            mModuleRotations1Entry.setDouble(drivetrain.getModuleRotations()[0]);
            mModuleRotations2Entry.setDouble(drivetrain.getModuleRotations()[1]);
            mModuleRotations3Entry.setDouble(drivetrain.getModuleRotations()[2]);
            mModuleRotations4Entry.setDouble(drivetrain.getModuleRotations()[3]);

            mOdometryXEntry.setDouble(drivetrain.getPose().getX());
            mOdometryYEntry.setDouble(drivetrain.getPose().getY());
            mOdometryThetaEntry.setDouble(drivetrain.getPose().getRotation().getDegrees());
            /*
             * Adjustable Drivetrain PID below, DO THIS LATER, copied in the flywheel pid toggle tentative logic that I didn't double check
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
        } catch (IllegalArgumentException e) {}
    }
}
