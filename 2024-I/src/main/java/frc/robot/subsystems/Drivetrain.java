package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.RollingAverage;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LimelightHelper;

public class Drivetrain extends SubsystemBase {

    public static Drivetrain instance;

    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;
    private SwerveDrivePoseEstimator odometry;

    private final Pigeon2 gyro;
    private double heading;
    private double currentDrivetrainSpeed = 0;
    private ChassisSpeeds currentRobotRelativeSpeed;

    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;

    private RollingAverage gyroTiltAverage;
    private Field2d field;

    private static double inch2meter(double inch) {
        return inch * 0.0254;
    }

    private double S2 = 18.0;
    private double I2 = 0.1;
    private double K2 = 4.0;
    private double H2 = 3.3;
    private double sigmoid2(double dist) {
        return (S2-I2)/(1.0+Math.exp(-K2*(dist-H2)))+I2;
    }

    private double S3 = 5.0;
    private double I3 = 0.1;
    private double K3 = 3.0;
    private double H3 = 3.3;
    private double sigmoid3(double dist) {
        return (S3-I3)/(1.0+Math.exp(-K3*(dist-H3)))+I3;
    }

    private boolean useMegaTag;
    private boolean isForcingCalibration;

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

    public Drivetrain() {
        frontLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID, DriveConstants.kFrontLeftModuleAngularOffset);
        frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID, DriveConstants.kFrontRightModuleAngularOffset);
        backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID, DriveConstants.kBackLeftModuleAngularOffset);
        backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID, DriveConstants.kBackRightModulelAngularOffset);

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(),
                frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() };

        gyro = new Pigeon2(RobotMap.GYRO, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);
        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, gyro.getRotation2d(), swerveModulePositions,
                new Pose2d());

        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();
        
        SmartDashboard.putBoolean("Reset Gyro", false);

        SmartDashboard.putNumber("Logistic S2", S2); 
        SmartDashboard.putNumber("Logistic I2", I2); 
        SmartDashboard.putNumber("Logistic K2", K2); 
        SmartDashboard.putNumber("Logistic H2", H2);
        
        SmartDashboard.putNumber("Logistic S3", S3); 
        SmartDashboard.putNumber("Logistic I3", I3); 
        SmartDashboard.putNumber("Logistic K3", K3); 
        SmartDashboard.putNumber("Logistic H3", H3); 
        
        SmartDashboard.putBoolean("Megatag updates", true);
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        S2 = SmartDashboard.getNumber("Logistic S2", S2); 
        I2 = SmartDashboard.getNumber("Logistic I2", I2); 
        K2 = SmartDashboard.getNumber("Logistic K2", K2); 
        H2 = SmartDashboard.getNumber("Logistic H2", H2); 

        S3 = SmartDashboard.getNumber("Logistic S3", S3); 
        I3 = SmartDashboard.getNumber("Logistic I3", I3); 
        K3 = SmartDashboard.getNumber("Logistic K3", K3); 
        H3 = SmartDashboard.getNumber("Logistic H3", H3); 

        useMegaTag = SmartDashboard.getBoolean("Megatag updates", useMegaTag);

        field.setRobotPose(getPose()); 
        SmartDashboard.putData(field); 

        // Updating the odometry
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
            swerveModules[i].putSmartDashboard();
        }

        double distance = inch2meter(limelightFront.getDistance());
        int numAprilTag = LimelightHelper.getNumberOfAprilTagsSeen(limelightFront.getLimelightName());

        if (numAprilTag >= 2) {
            double stdDev = isForcingCalibration ? 0.0001 : (numAprilTag >= 3 ? sigmoid3(distance) : sigmoid2(distance));
            
            SmartDashboard.putNumber("distance", distance);
            SmartDashboard.putNumber("standard deviation", stdDev);

            Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDev, stdDev, isForcingCalibration ? 0.0001 : 30);
            odometry.setVisionMeasurementStdDevs(visionStdDevs);
        }
        
        updateModulePositions();
        updateOdometry();

        SmartDashboard.putNumber("Gyro Angle", getHeading());
        for(SwerveModule m:swerveModules) m.updateSmartdashBoard();
        if(SmartDashboard.getBoolean("Reset Gyro", false)){
            gyro.setYaw(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void updateModulePositions() {
        for (int i = 0; i < swerveModulePositions.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
        }
    }

    public void updateOdometry() {
        odometry.update(getRotation2d(), swerveModulePositions);
        if (useMegaTag){
            limelightFront.checkForAprilTagUpdates(odometry);
            limelightBack.checkForAprilTagUpdates(odometry);
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

        if (fieldOriented) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getRotation2d());
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2) + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));
        currentRobotRelativeSpeed = robotRelativeSpeeds; //not sure if robot relative

        SmartDashboard.putNumber("Chassis Speed X", robotRelativeSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speed Y", robotRelativeSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Theta", robotRelativeSpeeds.omegaRadiansPerSecond);

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxAngularSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    // Autonomous Drive Functions

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        optimizeModuleStates();
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

    // Returns the current pose of the robot
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void resetPose(Pose2d pose){
        resetGyro();
        odometry.resetPosition(getRotation2d(), swerveModulePositions, pose);
    }

    public double getSpeed(){
        return currentDrivetrainSpeed;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kinematics.toChassisSpeeds(swerveModuleStates);
    }

    public SwerveModuleState[] getSwerveModuleState(){
        return swerveModuleStates;
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        return swerveModulePositions;
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double[] getModuleRotations(){
        double[] positions = {frontLeftModule.getRotations(),
            backLeftModule.getRotations(),
            frontRightModule.getRotations(),
            backRightModule.getRotations()};
        return (positions);
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }

}