package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;
import frc.robot.utils.Constants.DriveConstants;

public class DebugTab extends ShuffleboardTabBase{

    private GenericEntry swerveModule1OffsetEntry, swerveModule2OffsetEntry, swerveModule3OffsetEntry, swerveModule4OffsetEntry;

    public DebugTab(){
    }

    public void createEntries() {
        tab = Shuffleboard.getTab("Operator");

        try{ 
            swerveModule1OffsetEntry = tab.add("Front Left Offset", 0.0)
            .withSize(2,1)
            .withPosition(0,0)
            .getEntry();    

            swerveModule2OffsetEntry = tab.add("Front Right Offset", 0.0)
            .withSize(1,1)
            .withPosition(0,2)
            .getEntry();   
            
            swerveModule3OffsetEntry = tab.add("Back Left Offset", 0.0)
            .withSize(2,1)
            .withPosition(0,4)
            .getEntry();   

            swerveModule4OffsetEntry = tab.add("Back Right Offset", 0.0)
            .withSize(2,1)
            .withPosition(0,6)
            .getEntry();   
        } catch (IllegalArgumentException e){
        }
    }

    @Override
    public void update() { //Some lines here are arbitrary code that should be implemented later but don't have the necessary methods in our subsystems right now.
        try {
            swerveModule1OffsetEntry.setDouble(DriveConstants.kFrontLeftModuleAngularOffset);
            swerveModule2OffsetEntry.setDouble(DriveConstants.kFrontRightModuleAngularOffset);
            swerveModule3OffsetEntry.setDouble(DriveConstants.kBackLeftModuleAngularOffset);
            swerveModule4OffsetEntry.setDouble(DriveConstants.kBackRightModulelAngularOffset);
        } catch(IllegalArgumentException e){}
    }
}