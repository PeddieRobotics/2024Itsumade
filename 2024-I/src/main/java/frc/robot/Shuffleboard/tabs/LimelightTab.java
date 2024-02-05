package frc.robot.Shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Limelight;

public class LimelightTab extends ShuffleboardTabBase{

    private Limelight limelight = Limelight.getInstance();

    private GenericEntry limelightHasTargetEntry, limelightTagsSeenEntry,
    mBotposeAEntry, mBotposeXEntry, mBotposeYEntry, 
    mDistToTargetEntry, mTaEntry, mTxEntry, mTyEntry;

    public LimelightTab(){}

    public void createEntries(){
        tab = Shuffleboard.getTab("LimelightTab");

        try{
            limelightHasTargetEntry = tab.add("Has Target", false)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            limelightTagsSeenEntry = tab.add("Tags Seen", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mBotposeAEntry = tab.add("Botpose Theta", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mBotposeXEntry = tab.add("Botpose X", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mBotposeYEntry = tab.add("Botpose Y", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mDistToTargetEntry = tab.add("Distance to Target", 0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mTaEntry = tab
            .add("ta", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mTxEntry = tab
            .add("tx", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();

            mTyEntry = tab
            .add("ty", 0.0)
            .withSize(2, 2)
            .withPosition(4, 5)
            .getEntry();
        } catch (IllegalArgumentException e){}
    }

    @Override
    public void update(){
        try{
        /*  mTxEntry.setDouble(LLBack.getTx());
            mTyEntry.setDouble(LLBack.getTy());
            mTaEntry.setDouble(LLBack.getTa());
            limelightHasTargetEntry.setBoolean(LLBack.hasTarget());
            mDistToTargetEntry.setDouble(LLBack.getDistance());
            limelightTagsSeenEntry.setInteger(LLBack.getTagsSeen());
            mBotposeXEntry.setDouble(LLBack.getBotpose().getX());
            mBotposeYEntry.setDouble(LLBack.getBotpose().getY());
            mBotposeAEntry.setDouble(LLBack.getBotpose().getRotation().getDegrees()); */
        } catch (IllegalArgumentException e){}
    }
}
