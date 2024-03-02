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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;

public class Logger {
    private static Logger instance;
    private BooleanLogEntry intakeSensorEntry, hopperBottomSensorEntry, hopperTopSensorEntry;
    private DoubleLogEntry gyroAngleEntry, drivetrainSpeedEntry, intakeCurrentEntry, hopperCurrentEntry, 
                leftFlywheelCurrentEntry,rightFlywheelCurrentEntry,armAngleEntry,leftFlywheelRPMEntry,rightFlywheelRPMEntry,LLDistancEntry;
    private StringLogEntry robotStateEntry, commandEntry;
    private DoubleArrayLogEntry fieldPositionEntry, moduleSpeedsEntry, modulePositionsEntry;
    private DataLog log = DataLogManager.getLog();
    private double lastTeleopEnable;
    private Pose2d fieldPosition;

    private Drivetrain drivetrain;
    private Intake intake;
    private Flywheel flywheel;
    private Arm arm;
    private LimelightShooter limelightShooter;
    // private Hopper hopper;
    // private Superstructure superstructure;

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
        limelightShooter=LimelightShooter.getInstance();
        // hopper = Hopper.getInstance();
        // superstructure = Superstructure.getInstance();

        // Superstructure Logs
        robotStateEntry = new StringLogEntry(log, "/Superstructure/Robot State");

        // Drivetrain Logs
        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        drivetrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        fieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Postion");
        moduleSpeedsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Positions");

        // Intake Logs
        intakeSensorEntry = new BooleanLogEntry(log, "/Intake/Intake Sensor");
        intakeCurrentEntry = new DoubleLogEntry(log, "/Intake/Intake Current");

        // Hopper Logs
        // hopperBottomSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Top Sensor");
        // hopperTopSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Top Sensor");
        // hopperCurrentEntry = new DoubleLogEntry(log, "/Hopper/Hopper Current");

        //flywheel Loges
        rightFlywheelCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Right Motor Current");
        leftFlywheelCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Left Motor Current");

        rightFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Right RPM");
        leftFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Left RPM");

        //arm logs
        armAngleEntry = new DoubleLogEntry(log, "/Arm/Angle Degrees");

        //LL logs
        LLDistancEntry = new DoubleLogEntry(log, "/Limelight/Distance");

        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, Boolean isStart) {
        commandEntry.append(event + (isStart? " Started": " Ended"));

    }

    public void updateLogs() {
        // Superstructure
        // robotStateEntry.append(superstructure.getRobotState());

        // Drivetrain
        updateDrivetrainLogs();

        // Intake
        intakeSensorEntry.append(intake.getSensor());
        intakeCurrentEntry.append(intake.getMotorCurrent());

        // Hopper
        // hopperBottomSensorEntry.append(hopper.bottomSensor());
        // hopperTopSensorEntry.append(hopper.topSensor());
        // hopperCurrentEntry.append(hopper.getMotorCurrent());

        //Flywheel
        rightFlywheelCurrentEntry.append(flywheel.getRightMotorCurrent());
        leftFlywheelCurrentEntry.append(flywheel.getLeftMotorCurrent());

        rightFlywheelRPMEntry.append(flywheel.getFlywheelRightRPM());
        leftFlywheelRPMEntry.append(flywheel.getFlywheelLeftRPM());

        //arm
        armAngleEntry.append(arm.getArmAngleDegrees());

        //limelight
        LLDistancEntry.append(limelightShooter.getDistance());
    }

    //put pose to log for field visualization
    //put swerve module speeds and positions to log
    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(drivetrain.getHeading());
        drivetrainSpeedEntry.append(drivetrain.getSpeed());

        double[] pose = { drivetrain.getPose().getX(), drivetrain.getPose().getY(),
                drivetrain.getPose().getRotation().getDegrees() };
        fieldPositionEntry.append(pose);

        SwerveModuleState[] moduleStates = drivetrain.getSwerveModuleState();
        double[] swerveModulePositions = { moduleStates[0].angle.getDegrees(), moduleStates[1].angle.getDegrees(),
                moduleStates[2].angle.getDegrees(), moduleStates[3].angle.getDegrees() };
        double[] swerveModuleSpeeds = { moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond };
        modulePositionsEntry.append(swerveModulePositions);
        moduleSpeedsEntry.append(swerveModuleSpeeds);
    }

    public void signalRobotEnable() {
        lastTeleopEnable = Timer.getFPGATimestamp();
    }
}
