package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightBack extends Limelight {
    private static LimelightBack limelight;

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

    @Override
    public boolean hasGamepiece() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasGamepiece'");
    }

    public void forceAprilTagLocalization(SwerveDrivePoseEstimator odometry) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'forceAprilTagLocalization'");
    }

    public Pose2d getBotPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBotPose'");
    }

    public static LimelightBack getInstance(){
        if(limelight == null){
            limelight = new LimelightBack();
        }
        return limelight;
    }
}
