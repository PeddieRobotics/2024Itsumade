package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Logger;

public class Drivetrain extends SubsystemBase {

    public static Drivetrain instance;

    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;
    private SwerveDrivePoseEstimator odometry;

    private final Pigeon2 gyro;
    private double heading;
    private double autoAdjustAngle = 0;
    private double currentDrivetrainSpeed = 0;
    private ChassisSpeeds currentRobotRelativeSpeed;

    private Timer timer;
    private double previousTime;
    private double offTime;
    private Rotation2d holdHeading;
    private final LimelightShooter limelightShooter;
    private final LimelightIntake limelightIntake;

    private Field2d field;

    private Pose2d[] poseHistory;
    private double[] gyroHistory;

    // logistic function for if two apriltags are seen
    private double S2 = 18.0; // maximum
    private double I2 = 0.1; // minimum
    private double K2 = 4.0; // growth rate
    private double H2 = 3.3; // midpoint

    private double redPassingGyroAngle;
    private double bluePassingGyroAngle;

    private double sigmoid2(double dist) {
        return (S2 - I2) / (1.0 + Math.exp(-K2 * (dist - H2))) + I2;
    }

    // logistic function for if three apriltags are seen
    private double S3 = 5.0;
    private double I3 = 0.1;
    private double K3 = 3.0;
    private double H3 = 3.3;

    private double sigmoid3(double dist) {
        return (S3 - I3) / (1.0 + Math.exp(-K3 * (dist - H3))) + I3;
    }

    // turning the usage of vision updates on or off, typically for autos
    // automatically turned off in autonomousInit, turned on in teleopInit
    private boolean useMegaTag;

    // forcible set the robot odom completely based on megatag botpose
    private boolean isForcingCalibration;

    // make the robot not move, because if there's no note at the midline we park
    // automatically turned off in autonomousInit and teleopInit
    private boolean isParkedAuto;

    private double stdDev;
    private int numAprilTag;

    public Drivetrain() {
        frontLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kFrontLeftModuleAngularOffset);
        frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kFrontRightModuleAngularOffset);
        backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kBackLeftModuleAngularOffset);
        backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kBackRightModulelAngularOffset);

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(),
                frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() };
        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        gyro = new Pigeon2(RobotMap.GYRO, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);
        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, gyro.getRotation2d(), swerveModulePositions,
                new Pose2d());
        field = new Field2d();

        poseHistory = new Pose2d[10]; // Keep track of the last 20ms * 10 = 200 ms of pose history
        gyroHistory = new double[10]; // Keep track of the last 20ms * 10 = 200 ms of gyro history

        timer = new Timer();
        timer.start();
        previousTime = 0.0;
        offTime = 0.0;

        isParkedAuto = false;

        autoAdjustAngle = 0.0;

        limelightShooter = LimelightShooter.getInstance();
        limelightIntake = LimelightIntake.getInstance();

        redPassingGyroAngle = LimelightConstants.kRedCornerPassingGyroAngle;
        bluePassingGyroAngle = LimelightConstants.kBlueCornerPassingGyroAngle;

        // SmartDashboard.putBoolean("Reset Gyro", false);

        // constant for logistic function scaling the vision std based on distance
        // SmartDashboard.putNumber("Logistic S2", S2);
        // SmartDashboard.putNumber("Logistic I2", I2);
        // SmartDashboard.putNumber("Logistic K2", K2);
        // SmartDashboard.putNumber("Logistic H2", H2);

        // SmartDashboard.putNumber("Logistic S3", S3);
        // SmartDashboard.putNumber("Logistic I3", I3);
        // SmartDashboard.putNumber("Logistic K3", K3);
        // SmartDashboard.putNumber("Logistic H3", H3);

        // SmartDashboard.putBoolean("Megatag updates", true);

        // // temp
        // SmartDashboard.putNumber("Target P", 0);
        // SmartDashboard.putNumber("Target I", 0);
        // SmartDashboard.putNumber("Target D", 0);
        // SmartDashboard.putNumber("Target FF", 0);

        // SmartDashboard.putNumber("Correct Heading P", DriveConstants.kHeadingCorrectionP);

        isForcingCalibration = false;
        useMegaTag = Constants.DriveConstants.kUseMegaTag;
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // updatePoseHistory();
        updateGyroHistory();
        
        // S2 = SmartDashboard.getNumber("Logistic S2", S2);
        // I2 = SmartDashboard.getNumber("Logistic I2", I2);
        // K2 = SmartDashboard.getNumber("Logistic K2", K2);
        // H2 = SmartDashboard.getNumber("Logistic H2", H2);

        // S3 = SmartDashboard.getNumber("Logistic S3", S3);
        // I3 = SmartDashboard.getNumber("Logistic I3", I3);
        // K3 = SmartDashboard.getNumber("Logistic K3", K3);
        // H3 = SmartDashboard.getNumber("Logistic H3", H3);

        // useMegaTag = SmartDashboard.getBoolean("Megatag updates", useMegaTag);

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);

        // Updating the odometry
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
            swerveModules[i].putSmartDashboard();
        }

        double distance = Units.inchesToMeters(limelightShooter.getDistance());
        numAprilTag = LimelightHelper.getNumberOfAprilTagsSeen(limelightShooter.getLimelightName());

        if (numAprilTag >= 2) {
            // if forcing calibration make visionstd minimal otherwise choose between
            // function for 3 and 2 based on number of tags seen
            stdDev = isForcingCalibration ? 0.0001
                    : (numAprilTag >= 3 ? sigmoid3(distance) : sigmoid2(distance));

            Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDev, stdDev, isForcingCalibration ? 0.0001 : 30);
            odometry.setVisionMeasurementStdDevs(visionStdDevs);
        }

        updateModulePositions();
        updateOdometry();

        // SmartDashboard.putNumber("Gyro Angle", getHeading());
        // for (SwerveModule m : swerveModules)
        // m.updateSmartdashBoard();
        // if (SmartDashboard.getBoolean("Reset Gyro", false)) {
        // gyro.setYaw(0);
        // }
        SmartDashboard.putNumber("Gyro heading", getHeading());
        SmartDashboard.putNumber("Odometry X", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry Theta", odometry.getEstimatedPosition().getRotation().getDegrees());
    }

    public void updateModulePositions() {
        for (int i = 0; i < swerveModulePositions.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
        }
    }

    public void updateOdometry() {
        odometry.update(getHeadingAsRotation2d(), swerveModulePositions);
        //Logger.getInstance().logEvent("isForcingCalibration variable", isForcingCalibration);
        //Logger.getInstance().logEvent("useMegaTag variable", useMegaTag);
        if(DriverStation.isAutonomous()){
            if (isForcingCalibration) {
                limelightShooter.checkForAprilTagUpdates(odometry);
            }
        }
        else{
            if (useMegaTag || isForcingCalibration) {
                limelightShooter.checkForAprilTagUpdates(odometry);
                isForcingCalibration = false;
            }
        }
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented,
            Translation2d centerOfRotation) {
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        ChassisSpeeds robotRelativeSpeeds;

        SmartDashboard.putNumber("Auto adjust angle", autoAdjustAngle);

        if (fieldOriented) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d().plus(new Rotation2d(Math.toRadians(autoAdjustAngle))));
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));
        currentRobotRelativeSpeed = robotRelativeSpeeds; // not sure if robot relative
        robotRelativeSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);// real loop time should be .02

        // fudge factoring

        double fudgefactor = -.11;// -.11
        Translation2d commandedVelocity = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond);
        Rotation2d commandedRotation = Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond);
        Translation2d TangentVelocity = commandedVelocity.rotateBy(Rotation2d.fromDegrees(90));
        commandedVelocity = commandedVelocity.plus(TangentVelocity.times(fudgefactor * commandedRotation.getRadians())); // adds
                                                                                                                         // tangent
                                                                                                                         // veclocity
                                                                                                                         // times
                                                                                                                         // rotational
                                                                                                                         // speed
                                                                                                                         // times
                                                                                                                         // fudge
                                                                                                                         // factor
        robotRelativeSpeeds = new ChassisSpeeds(commandedVelocity.getX(), commandedVelocity.getY(),
                commandedRotation.getRadians());

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxAngularSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    private ChassisSpeeds correctHeading(ChassisSpeeds currentSpeeds) {
        double currentTime = timer.get();
        double dt = currentTime - previousTime;

        double vTheta = currentSpeeds.omegaRadiansPerSecond;
        double vTranslation = Math
                .sqrt(Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2));

        if (Math.abs(vTheta) > 0.01) {
            offTime = currentTime;
            holdHeading = getHeadingAsRotation2d();
            return currentSpeeds;
        }
        if (currentTime - offTime < 0.5) {
            holdHeading = getHeadingAsRotation2d();
            return currentSpeeds;
        }
        if (vTranslation < 0.1) {
            holdHeading = getHeadingAsRotation2d();
            return currentSpeeds;
        }

        holdHeading = holdHeading.plus(new Rotation2d(vTheta * dt));

        Rotation2d deltaHeading = holdHeading.minus(getHeadingAsRotation2d());

        if (Math.abs(deltaHeading.getDegrees()) < DriveConstants.kHeadingCorrectionTolerance) {
            return currentSpeeds;
        }

        double correctedVTheta = deltaHeading.getRadians() / dt
                * SmartDashboard.getNumber("Correct Heading P", DriveConstants.kHeadingCorrectionP);
        previousTime = currentTime;

        return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, correctedVTheta);
    }

    // Autonomous Drive Functions

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        if (isParkedAuto)
            return;
        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        // optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    public void optimizeModuleStates() {
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i] = SwerveModuleState.optimize(swerveModuleStates[i],
                    new Rotation2d(swerveModules[i].getCANCoderReading()));
        }
    }

    public void stop() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public double getHeading() {
        heading = gyro.getAngle();
        return Math.IEEEremainder(heading, 360);
    }

    public double getAdjustedGyroHeading(){
        return getHeadingAsRotation2d().plus(new Rotation2d(Math.toRadians(autoAdjustAngle))).getDegrees();
    }

    public Rotation2d getHeadingAsRotation2d() {
        Rotation2d rotation = gyro.getRotation2d();
        // return rotation.times(-1.0);
        return rotation;
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        gyro.reset();
        odometry.resetPosition(getHeadingAsRotation2d(), swerveModulePositions, pose);
        
    }

    public double getSpeed() {
        return currentDrivetrainSpeed;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {

        return DriveConstants.kinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(),
                backLeftModule.getState(), backRightModule.getState());
    }

    public SwerveModuleState[] getSwerveModuleState() {
        return swerveModuleStates;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveModulePositions;
    }

    public void resetGyro() {
        autoAdjustAngle = 180;
        resetPose(odometry.getEstimatedPosition());
    }

    public double getGyroXAcceleration(){
        return gyro.getAccelerationX().getValueAsDouble();
    }

    public double getGyroYAcceleration(){
        return gyro.getAccelerationY().getValueAsDouble();
    }

    public double getGyroZAcceleration(){
        return gyro.getAccelerationZ().getValueAsDouble();
    }

    public double[] getModuleRotations() {
        double[] positions = { frontLeftModule.getRotations(),
                backLeftModule.getRotations(),
                frontRightModule.getRotations(),
                backRightModule.getRotations() };
        return (positions);
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }

    public void setAutoAdjustAngle(double angleOffset){
        autoAdjustAngle = angleOffset;
    }

    public double getAutoAdjustAngle(){
        return autoAdjustAngle;
    }

    public void setRedPassingGyroAngle(double angle){
        redPassingGyroAngle = angle;
    }

    public double getRedPassingGyroAngle(){
        return redPassingGyroAngle;
    }

    public void setBluePassingGyroAngle(double angle){
        bluePassingGyroAngle = angle;
    }

    public double getBluePassingGyroAngle(){
        return bluePassingGyroAngle;
    }

    public double getPastHeading(double latency){
        int latencyIndexToNearestTwentyMS = (int)(Math.round(latency/20.0)) - 1;
        return gyroHistory[latencyIndexToNearestTwentyMS];
    }

    public double getStandardDeviation() {
        return stdDev;
    }
    public int getNumApriltags() {
        return numAprilTag;
    }
    

    public boolean getUseMegaTag() {
        return useMegaTag;
    }

    public void setUseMegaTag(boolean useMegaTag) {
        this.useMegaTag = useMegaTag;
    }

    public boolean getIsForcingCalibration() {
        return isForcingCalibration;
    }

    public void setIsForcingCalibration(boolean isForcingCalibration) {
        this.isForcingCalibration = isForcingCalibration;
    }

    public boolean getIsParkedAuto() {
        return isParkedAuto;
    }

    public void setIsParkedAuto(boolean isParked) {
        this.isParkedAuto = isParked;
    }


    
    public void updatePoseHistory(){
        for(int i = 0; i < poseHistory.length-1; i++){
            poseHistory[i+1] = poseHistory[i];
        }
        poseHistory[0] = odometry.getEstimatedPosition();
    }

    public void updateGyroHistory(){
        for(int i = 0; i < gyroHistory.length-1; i++){
            gyroHistory[i+1] = gyroHistory[i];
        }
        gyroHistory[0] = getHeading();
    }

    public double getFrontLeftDriveSupplyCurrent() {
        return frontLeftModule.getDriveMotorSupplyCurrent();
    }

    public double getFrontLeftSteerSupplyCurrent() {
        return frontLeftModule.getSteerMotorSupplyCurrent();
    }

    public double getFrontRightDriveSupplyCurrent() {
        return frontRightModule.getDriveMotorSupplyCurrent();
    }

    public double getFrontRightSteerSupplyCurrent() {
        return frontRightModule.getSteerMotorSupplyCurrent();
    }

    public double getBackLeftDriveSupplyCurrent() {
        return backLeftModule.getDriveMotorSupplyCurrent();
    }

    public double getBackLeftSteerSupplyCurrent() {
        return backLeftModule.getSteerMotorSupplyCurrent();
    }

    public double getBackRightDriveSupplyCurrent() {
        return backRightModule.getDriveMotorSupplyCurrent();
    }

    public double getBackRightSteerSupplyCurrent() {
        return backRightModule.getSteerMotorSupplyCurrent();
    }
    

    public double getFrontLeftDriveStatorCurrent() {
        return frontLeftModule.getDriveMotorStatorCurrent();
    }
    
    public double getFrontLeftSteerStatorCurrent() {
        return frontLeftModule.getSteerMotorStatorCurrent();
    }
    
    public double getFrontRightDriveStatorCurrent() {
        return frontRightModule.getDriveMotorStatorCurrent();
    }
    
    public double getFrontRightSteerStatorCurrent() {
        return frontRightModule.getSteerMotorStatorCurrent();
    }
    
    public double getBackLeftDriveStatorCurrent() {
        return backLeftModule.getDriveMotorStatorCurrent();
    }
    
    public double getBackLeftSteerStatorCurrent() {
        return backLeftModule.getSteerMotorStatorCurrent();
    }
    
    public double getBackRightDriveStatorCurrent() {
        return backRightModule.getDriveMotorStatorCurrent();
    }
    
    public double getBackRightSteerStatorCurrent() {
        return backRightModule.getSteerMotorStatorCurrent();
    }
}