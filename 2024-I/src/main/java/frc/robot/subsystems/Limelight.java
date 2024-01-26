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


}
