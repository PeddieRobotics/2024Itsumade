package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
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

    public Drivetrain() {
        frontLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID);
        frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID);
        backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID);
        backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID);

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(),
                frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() };

        gyro = new Pigeon2(RobotMap.GYRO, RobotMap.CANIVORE_NAME);
        odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, gyro.getRotation2d(), swerveModulePositions,
                new Pose2d());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxFloorSpeed);

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

        swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxAngularSpeed);
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