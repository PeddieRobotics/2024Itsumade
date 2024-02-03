package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RollingAverage;

public class LimelightBack extends Limelight {
    private static LimelightBack limelight;

    private RollingAverage txAverage, tyAverage, taAverage, xAverage, rotationAverage, rxAverage, ryAverage;

    //TODO: set name
    private String limelightName = "";

    public LimelightBack(){
        
    }

    public boolean hasTarget(){
        return false;
    }

    public int getPipeline(){
        return 0;
    }

    public String getJSONDump() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getJSONDump'");
    }

    public double getTxAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTxAverage'");
    }

    public double getTaAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTaAverage'");
    }

    public void forceAprilTagLocalization(SwerveDrivePoseEstimator odometry) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'forceAprilTagLocalization'");
    }

    public Pose2d getBotPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBotPose'");
    }

    @Override
    public double getTyAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTyAverage'");
    }

    @Override
    public double getRotationAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotationAverage'");
    }

    @Override
    public double getRXAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRXAverage'");
    }

    @Override
    public double getRYAverage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRYAverage'");
    }

    public static LimelightBack getInstance(){
        if(limelight == null){
            limelight = new LimelightBack();
        }
        return limelight;
    }
}