package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.DriveConstants;

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

        SmartDashboard.putBoolean("Reset Gyro", false);
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateModulePositions();
        updateOdometry();
        SmartDashboard.putNumber("Gyro Angle", getHeading());
        for (SwerveModule m : swerveModules)
            m.updateSmartdashBoard();
        if (SmartDashboard.getBoolean("Reset Gyro", false)) {
            gyro.setYaw(0);
        }
        SmartDashboard.putNumber("Odometry X", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry Theta", odometry.getEstimatedPosition().getRotation().getDegrees());
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

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));
        currentRobotRelativeSpeed = robotRelativeSpeeds; // not sure if robot relative

        SmartDashboard.putNumber("Chassis Speed X", robotRelativeSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Speed Y", robotRelativeSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Theta", robotRelativeSpeeds.omegaRadiansPerSecond);

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxAngularSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    // Autonomous Drive Functions

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
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

    public Rotation2d getRotation2d() {
        Rotation2d rotation = gyro.getRotation2d();
        // return rotation.times(-1.0);
        return rotation;
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        resetGyro();
        odometry.resetPosition(getRotation2d(), swerveModulePositions, pose);
    }

    public double getSpeed() {
        return currentDrivetrainSpeed;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());
    }

    public SwerveModuleState[] getSwerveModuleState() {
        return swerveModuleStates;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return swerveModulePositions;
    }

    public void resetGyro() {
        gyro.reset();
    }
}