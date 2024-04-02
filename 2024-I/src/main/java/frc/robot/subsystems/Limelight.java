package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Limelight extends SubsystemBase {
    public abstract boolean hasTarget();

    public abstract int getPipeline();

    public abstract String getJSONDump();

    public abstract double getTxAverage();

    public abstract double getTyAverage();

    public abstract double getTaAverage();

    public abstract double getRotationAverage();

    public abstract double getRXAverage(); 

    public abstract double getRYAverage(); 

    public abstract Pose2d getBotpose();
}
