package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;

public class Logger {
    private static Logger instance;
    private BooleanLogEntry intakeSensorEntry, hopperBottomSensorEntry, hopperTopSensorEntry, LLIntakeHasTargetEntry;
    private DoubleLogEntry gyroAngleEntry, adjustedGyroAngleEntry, drivetrainSpeedEntry, intakeCurrentEntry, intakeSpeedEntry, hopperCurrentEntry, 
                leftFlywheelSupplyCurrentEntry,rightFlywheelSupplyCurrentEntry,armAngleEntry,leftFlywheelRPMEntry,rightFlywheelRPMEntry,
                LLShooterDistanceEntry, LLShooterTxEntry, LLIntakeTxEntry, armSupplyCurrentEntry, armAngleSetpointEntry, climberLeftArmPosition, climberRightArmPosition,
                climberLeftArmCurrent, climberRightArmCurrent, LLShooterNumOfApriltagEntry, stdDevEntry, armTorqueCurrentEntry, rightFlywheelTorqueCurrentEntry, leftFlywheelTorqueCurrentEntry,
                gyroXAccelEntry, gyroYAccelEntry, gyroZAccelEntry;
    private DoubleLogEntry frontLeftDriveSupplyCurrent, frontLeftSteerSupplyCurrent, frontRightDriveSupplyCurrent,frontRightSteerSupplyCurrent,
                backLeftDriveSupplyCurrent,backLeftSteerSupplyCurrent,backRightDriveSupplyCurrent, backRightSteerSupplyCurrent;
    private DoubleLogEntry frontLeftDriveStatorCurrent, frontLeftSteerStatorCurrent, frontRightDriveStatorCurrent,frontRightSteerStatorCurrent,
                backLeftDriveStatorCurrent,backLeftSteerStatorCurrent,backRightDriveStatorCurrent, backRightSteerStatorCurrent;

    private StringLogEntry robotStateEntry, commandEntry, lightStateEntry;
    private DoubleArrayLogEntry fieldPositionEntry, botposeFieldPositionEntry, moduleSpeedsEntry, modulePositionsEntry;
    private DataLog log = DataLogManager.getLog();
    private double lastTeleopEnable;
    private Pose2d fieldPosition;

    private Drivetrain drivetrain;
    private Intake intake;
    private Flywheel flywheel;
    private Arm arm;
    private Climber climber;
    private LimelightShooter limelightShooter;
    private LimelightIntake limelightIntake;
    private Hopper hopper;
    private Superstructure superstructure;
    private Lights lights;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        intake = Intake.getInstance();
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        limelightShooter = LimelightShooter.getInstance();
        limelightIntake = LimelightIntake.getInstance();
        hopper = Hopper.getInstance();
        superstructure = Superstructure.getInstance();
        climber = Climber.getInstance();
        lights = Lights.getInstance();

        // Superstructure Logs
        robotStateEntry = new StringLogEntry(log, "/Superstructure/Robot State");

        // Drivetrain Logs
        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        adjustedGyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Adjusted Gyro Angle");

        gyroXAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro X Accel");
        gyroYAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Y Accel");
        gyroZAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Z Accel");

        drivetrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        fieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Position");
        botposeFieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Botpose position");
        moduleSpeedsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Positions");
        stdDevEntry = new DoubleLogEntry(log, "/Drivetrain/Megatag stddev");

        //swerve module current logs
        frontLeftDriveSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Left Drive Supply Current");
        frontLeftSteerSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Left Steer Supply Current");
        frontRightDriveSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Right Drive Supply Current");
        frontRightSteerSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Right Steer Supply Current");
        backLeftDriveSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Left Drive Supply Current");
        backLeftSteerSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Left Steer Supply Current");
        backRightDriveSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Right Drive Supply Current");
        backRightSteerSupplyCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Right Steer Supply Current");

        frontLeftDriveStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Left Drive Stator Current");
        frontLeftSteerStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Left Steer Stator Current");
        frontRightDriveStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Right Drive Stator Current");
        frontRightSteerStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Front Right Steer Stator Current");
        backLeftDriveStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Left Drive Stator Current");
        backLeftSteerStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Left Steer Stator Current");
        backRightDriveStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Right Drive Stator Current");
        backRightSteerStatorCurrent = new DoubleLogEntry(log, "/Drivetrain/Back Right Steer Stator Current");

        // Intake Logs
        intakeSensorEntry = new BooleanLogEntry(log, "/Intake/Intake Sensor");
        intakeCurrentEntry = new DoubleLogEntry(log, "/Intake/Intake Current");
        intakeSpeedEntry = new DoubleLogEntry(log, "/Intake/Intake Speed");

        // Hopper Logs
        hopperBottomSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Bottom Sensor");
        hopperTopSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Top Sensor");
        hopperCurrentEntry = new DoubleLogEntry(log, "/Hopper/Hopper Current");

        //flywheel Loges
        rightFlywheelSupplyCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Right Motor Supply Current");
        leftFlywheelSupplyCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Left Motor Supply Current");

        rightFlywheelTorqueCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Right Motor Torque Current");
        leftFlywheelTorqueCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Left Motor Torque Current");

        rightFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Right RPM");
        leftFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Left RPM");

        //arm logs
        armAngleEntry = new DoubleLogEntry(log, "/Arm/Angle Degrees");
        armSupplyCurrentEntry = new DoubleLogEntry(log, "/Arm/Supply Current");
        armAngleSetpointEntry = new DoubleLogEntry(log, "/Arm/Angle Setpoint");
        armTorqueCurrentEntry = new DoubleLogEntry(log, "/Arm/Torque Current");

        //climber logs
        climberLeftArmPosition = new DoubleLogEntry(log, "/Climber/Left Arm Motor Position");
        climberRightArmPosition = new DoubleLogEntry(log, "/Climber/Right Arm Motor Position");
        climberLeftArmCurrent = new DoubleLogEntry(log, "/Climber/Left Arm Motor Current");
        climberRightArmCurrent = new DoubleLogEntry(log, "/Climber/Right Arm Motor Current");

        //LL logs
        LLShooterDistanceEntry = new DoubleLogEntry(log, "/Limelight Shooter/Distance");
        LLShooterNumOfApriltagEntry = new DoubleLogEntry(log, "/Limelight Shooter/Number of Apriltags");
        LLShooterTxEntry = new DoubleLogEntry(log, "/Limelight Shooter/Tx");
        
        LLIntakeHasTargetEntry = new BooleanLogEntry(log, "/Limelight Intake/Has Target");
        LLIntakeTxEntry = new DoubleLogEntry(log, "/Limelight Intake/Tx");

        //commands used
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");

        //light states
        lightStateEntry = new StringLogEntry(log, "/Lights/Light State");
    }

    public void logEvent(String event, Boolean isStart) {
        commandEntry.append(event + (isStart? " Started": " Ended"));

    }

    public void updateLogs() {
        // Superstructure
        robotStateEntry.append(superstructure.getRobotState());

        // Lights
        lightStateEntry.append(lights.getLightStateAsString());

        // Drivetrain
        updateDrivetrainLogs();

        // Intake
        intakeSensorEntry.append(intake.getSensor());
        intakeCurrentEntry.append(intake.getMotorSupplyCurrent());
        intakeSpeedEntry.append(intake.getIntakeSpeed());

        // Hopper
        hopperBottomSensorEntry.append(hopper.getBottomSensor());
        hopperTopSensorEntry.append(hopper.getTopSensor());
        hopperCurrentEntry.append(hopper.getMotorSupplyCurrent());

        //Flywheel
        rightFlywheelSupplyCurrentEntry.append(flywheel.getRightMotorSupplyCurrent());
        leftFlywheelSupplyCurrentEntry.append(flywheel.getLeftMotorSupplyCurrent());

        rightFlywheelTorqueCurrentEntry.append(flywheel.getRightMotorTorqueCurrent());
        leftFlywheelTorqueCurrentEntry.append(flywheel.getLeftMotorTorqueCurrent());

        rightFlywheelRPMEntry.append(flywheel.getFlywheelRightRPM());
        leftFlywheelRPMEntry.append(flywheel.getFlywheelLeftRPM());

        //Arm
        armAngleEntry.append(arm.getArmAngleDegrees());
        armSupplyCurrentEntry.append(arm.getMotorSupplyCurrent());
        armAngleSetpointEntry.append(arm.getArmAngleSetpoint());
        armTorqueCurrentEntry.append(arm.getMotorTorqueCurrent());

        //limelight
        LLShooterDistanceEntry.append(limelightShooter.getDistance());
        LLShooterNumOfApriltagEntry.append(drivetrain.getNumApriltags());
        LLShooterTxEntry.append(limelightShooter.getTx());

        LLIntakeHasTargetEntry.append(limelightIntake.hasTarget());
        LLIntakeTxEntry.append(limelightIntake.getTx());

        //Climber
        climberLeftArmCurrent.append(climber.getLeftArmSupplyCurrent());
        climberRightArmCurrent.append(climber.getRightArmSupplyCurrent());

        climberLeftArmPosition.append(climber.getLeftArmPosition());
        climberRightArmPosition.append(climber.getRightArmPosition());

    }

    //put pose to log for field visualization
    //put swerve module speeds and positions to log
    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(drivetrain.getHeading());
        adjustedGyroAngleEntry.append(drivetrain.getAdjustedGyroHeading());

        gyroXAccelEntry.append(drivetrain.getGyroXAcceleration());
        gyroYAccelEntry.append(drivetrain.getGyroYAcceleration());
        gyroZAccelEntry.append(drivetrain.getGyroZAcceleration());

        drivetrainSpeedEntry.append(drivetrain.getSpeed());

        double[] pose = { drivetrain.getPose().getX(), drivetrain.getPose().getY(),
                drivetrain.getPose().getRotation().getDegrees() };
        fieldPositionEntry.append(pose);

        Pose2d botpose = limelightShooter.getCalculatedBotpose();
        if (botpose != null) {
            double[] botposePose = { botpose.getX(), botpose.getY(),
                            botpose.getRotation().getDegrees() };
            botposeFieldPositionEntry.append(botposePose);
        }

        SwerveModuleState[] moduleStates = drivetrain.getSwerveModuleState();
        double[] swerveModulePositions = { moduleStates[0].angle.getDegrees(), moduleStates[1].angle.getDegrees(),
                moduleStates[2].angle.getDegrees(), moduleStates[3].angle.getDegrees() };
        double[] swerveModuleSpeeds = { moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond };
        modulePositionsEntry.append(swerveModulePositions);
        moduleSpeedsEntry.append(swerveModuleSpeeds);

        stdDevEntry.append(drivetrain.getStandardDeviation());

        //Swerve modules
        //supply currents
        frontLeftDriveSupplyCurrent.append(drivetrain.getFrontLeftDriveSupplyCurrent());
        frontLeftSteerSupplyCurrent.append(drivetrain.getFrontLeftSteerSupplyCurrent());
        
        frontRightDriveSupplyCurrent.append(drivetrain.getFrontRightDriveSupplyCurrent());
        frontRightSteerSupplyCurrent.append(drivetrain.getFrontRightSteerSupplyCurrent());
        
        backLeftDriveSupplyCurrent.append(drivetrain.getBackLeftDriveSupplyCurrent());
        backLeftSteerSupplyCurrent.append(drivetrain.getBackLeftSteerSupplyCurrent());
        
        backRightDriveSupplyCurrent.append(drivetrain.getBackRightDriveSupplyCurrent());
        backRightSteerSupplyCurrent.append(drivetrain.getBackRightSteerSupplyCurrent());

        //stator currents
        frontLeftDriveStatorCurrent.append(drivetrain.getFrontLeftDriveStatorCurrent());
        frontLeftSteerStatorCurrent.append(drivetrain.getFrontLeftSteerStatorCurrent());

        frontRightDriveStatorCurrent.append(drivetrain.getFrontRightDriveStatorCurrent());
        frontRightSteerStatorCurrent.append(drivetrain.getFrontRightSteerStatorCurrent());

        backLeftDriveStatorCurrent.append(drivetrain.getBackLeftDriveStatorCurrent());
        backLeftSteerStatorCurrent.append(drivetrain.getBackLeftSteerStatorCurrent());

        backRightDriveStatorCurrent.append(drivetrain.getBackRightDriveStatorCurrent());
        backRightSteerStatorCurrent.append(drivetrain.getBackRightSteerStatorCurrent());

        SmartDashboard.putNumber("Standard Deviation from Logger", drivetrain.getStandardDeviation());
    }

    public void signalRobotEnable() {
        lastTeleopEnable = Timer.getFPGATimestamp();
    }
}
