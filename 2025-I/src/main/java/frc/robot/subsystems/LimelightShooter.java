package frc.robot.subsystems;

import org.opencv.calib3d.Calib3d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelper.LimelightResults;
import frc.robot.utils.LimelightHelper.LimelightTarget_Fiducial;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Logger;
import frc.robot.utils.RollingAverage;

public class LimelightShooter extends Limelight {
    private static LimelightShooter limelightShooter;

    private RollingAverage txAverage, tyAverage, taAverage, xAverage, rotationAverage, rxAverage, ryAverage;
    private LinearFilter distFilter;

    private double redTargetOffset, blueTargetOffset;

    private Pose2d calculatedBotpose;

    private String limelightName = "limelight-shooter";
    private double lastDistance;

    public LimelightShooter() {
        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();
        taAverage = new RollingAverage();
        rotationAverage = new RollingAverage();
        rxAverage = new RollingAverage(); 
        ryAverage = new RollingAverage(); 
        xAverage = new RollingAverage(4,getBotpose().getX());
        distFilter = LinearFilter.singlePoleIIR(0.24, 0.02);

        setPipeline(Constants.LimelightConstants.kShooterPipeline);
        lastDistance=0;
        redTargetOffset = LimelightConstants.kRedTargetTarget;
        blueTargetOffset = LimelightConstants.kBlueTargetTarget;
        // SmartDashboard.putNumber("Red Targeting Offset", redTargetOffset);
        // SmartDashboard.putNumber("Blue Targeting Offset", blueTargetOffset);
    }

    public static LimelightShooter getInstance() {
        if (limelightShooter == null) {
            limelightShooter = new LimelightShooter();
        }
        return limelightShooter;
    }

    public String getLimelightName(){
        return limelightName;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL Distance", getDistance());
        SmartDashboard.putNumber("LL Average Distance", getFilteredDistance());
        SmartDashboard.putNumber("LL ty", getTy());
        SmartDashboard.putNumber("LL tx", getTx());

        // setRedTargetingOffset(SmartDashboard.getNumber("Red Targeting Offset", redTargetOffset));
        // setBlueTargetingOffset(SmartDashboard.getNumber("Blue Targeting Offset", blueTargetOffset));
        //LimelightTarget_Fiducial[] fiducials = LimelightHelper.getLatestResults(limelightName).targetingResults.targets_Fiducials;
        //SmartDashboard.putNumber("LL results ty", fiducials[0].ty);
        //SmartDashboard.putNumber("LL results ty diff", fiducials[0].ty-getTy());
        if(hasTarget()) lastDistance=getDistance();
        updateRollingAverages();
    }

    public void startAveragingX(){
        xAverage = new RollingAverage(4,getBotpose().getX());
    }

    public double getAveragePoseX() {
        return xAverage.getAverage();
    }

    public Pose2d getBotpose() {
        double[] result;
        try {
            // Use blue coordinates by default for all odometry
            result = LimelightHelper.getBotPose_wpiBlue(limelightName);
            // if(DriverStation.getAlliance().get() == Alliance.Red){
            //     result = LimelightHelper.getBotPose_wpiRed(limelightName);
            // }
            // else{
            //     result = LimelightHelper.getBotPose_wpiBlue(limelightName);
            // }
        } catch (java.util.NoSuchElementException e) {
            return new Pose2d();
        }

        if (result.length > 0.0) {
            return new Pose2d(new Translation2d(result[0], result[1]), new Rotation2d(Math.toRadians(result[5])));
        }
        return new Pose2d();
    }

    
    // Tv is whether the limelight has a valid target
    public boolean getTv() {
        return LimelightHelper.getTV(limelightName);
    }

    // Tx is the Horizontal Offset From Crosshair To Target
    public double getTx() {
        double value = LimelightHelper.getTX(limelightName);
        return value;
    }

    public double getTx_NoCrosshair() {
        return LimelightHelper.getTX_NoCrosshair(limelightName);
    }

    // Ty is the Vertical Offset From Crosshair To Target
    public double getTy() {
        double value = LimelightHelper.getTY(limelightName);
        return value;
    }

    public double getTy_NoCrosshair() {
        return LimelightHelper.getTY_NoCrosshair(limelightName);
    }

    public double getTa() {
        return LimelightHelper.getTA(limelightName);
    }

    public double getTargetID(){
        return LimelightHelper.getFiducialID(limelightName);
    }

    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    public double getTaAverage() {
        return taAverage.getAverage();
    }

    public double getRotationAverage() {
        return rotationAverage.getAverage();
    }

    public double getRXAverage(){
        return rxAverage.getAverage(); 
    }

    public double getRYAverage(){
        return ryAverage.getAverage(); 
    }

    public void setRedTargetingOffset(double offset){
        redTargetOffset = offset;
    }

    public void setBlueTargetingOffset(double offset){
        blueTargetOffset = offset;
    }

    public double getRedTargetingOffset(){
        return redTargetOffset;
    }

    public double getBlueTargetingOffset(){
        return blueTargetOffset;
    }
    
    public Pose2d getCalculatedBotpose() {
        return calculatedBotpose;
    }

    // Class ID of primary neural detector result or neural classifier result
    public double getNeuralClassID() {
        return LimelightHelper.getNeuralClassID(limelightName);
    }

    public double getDistance() {
        if (!hasTarget()) {
            return 0;
        } else {
            // a1 = LL panning angle
            // a2 = additional angle to target
            // tan(a1 + a2) = h/d
            // d = h/tan(a1+a2)
            return (LimelightConstants.kSpeakerAprilTagHeight - LimelightConstants.kLimelightHeight) /
                    (Math.tan(Math.toRadians(LimelightConstants.kLimelightPanningAngle + getTy())));
        }
    }

    public double getFilteredDistance(){
        return distFilter.lastValue();
    }

    public int getTagsSeen() {
        return LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
    }

    public boolean hasTarget() {
        return getTv();
    }

    public void updateRollingAverages() {
        if (hasTarget()) {
            txAverage.add(getTx());
            tyAverage.add(getTy());
            taAverage.add(getTa());
            double dist = getDistance();
            if (dist != 0)
                distFilter.calculate(dist);
            
            // rotationAverage.add(getBotpose().getRotation().getDegrees());//based on alliance of driverstation, awaiting testing 
            rotationAverage.add(getBotpose().getRotation().getDegrees()); 

            //red
            rxAverage.add(getBotpose().getX()); 
            ryAverage.add(getBotpose().getY()); 

        }
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
        taAverage.clear();
        rotationAverage.clear(); 
        rxAverage.clear(); 
        ryAverage.clear(); 
    }

    public void setPipeline(int pipelineNum) {
        LimelightHelper.setPipelineIndex(limelightName, pipelineNum);
    }

    public int getPipeline(){
        return (int)LimelightHelper.getCurrentPipelineIndex(limelightName);
    }

    public String getJSONDump() {
        return LimelightHelper.getJSONDump(limelightName);
    }

    public void checkForAprilTagUpdates(SwerveDrivePoseEstimator odometry) {
        int tagsSeen = LimelightHelper.getNumberOfAprilTagsSeen(limelightName);
        //IMPORTANT:still has safe guard preventing the use of update vision if it is outside a half meter range, delete or change 
        //condition to furhter enable checkforapriltag updates 
        
        // if (tagsSeen > 1 && this.getBotpose().relativeTo(odometry.getEstimatedPosition()).getTranslation().getNorm() < 0.5) {
        //     odometry.addVisionMeasurement(this.getBotpose(), Timer.getFPGATimestamp());
        // }

        if (tagsSeen > 1) {
            // Get pipeline and capture latency (in milliseconds)
            double tl = LimelightHelper.getLatency_Pipeline(limelightName);
            double cl = LimelightHelper.getLatency_Capture(limelightName);
            // Calculate a latency-compensated timestamp for the vision measurement (in seconds)
            double timestampLatencyComp = Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0);
            calculatedBotpose = this.getBotpose();
            odometry.addVisionMeasurement(calculatedBotpose, timestampLatencyComp);
        }
        else
            calculatedBotpose = null;
    }
    
    // Gets the total latency of the limelight capture + pipeline processing for the current image, in milliseconds (MS)
    public double getTotalLatencyInMS(){
        double tl = LimelightHelper.getLatency_Pipeline(limelightName);
        double cl = LimelightHelper.getLatency_Capture(limelightName);
        return tl + cl;
    }

    public void setPriorityTag(int tagID){
        LimelightHelper.setPriorityTag(limelightName, tagID);
    }

    public double getLastDistance(){
        return lastDistance;
    }

    // TODO if use average distance then change to average distance
    public double getDistanceForLogging() {
        if (hasTarget())
            return getDistance();
        return getLastDistance();
    }
}