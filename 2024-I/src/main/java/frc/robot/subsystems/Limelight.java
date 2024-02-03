package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Limelight extends SubsystemBase {
    public abstract boolean hasTarget();

    public abstract int getPipeline();

    public abstract String getJSONDump();

    public abstract double getTxAverage();

    public abstract double getTaAverage();

    public abstract boolean hasGamepiece();

    public abstract void forceAprilTagLocalization(SwerveDrivePoseEstimator odometry);

    public abstract Pose2d getBotPose();

    public static Limelight getInstance() { //PLACEHOLDER METHOD, PLEASE PUT THIS IN THE LIMELIGHTFRONT CLASS AND NOT HERE
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
    }


}
