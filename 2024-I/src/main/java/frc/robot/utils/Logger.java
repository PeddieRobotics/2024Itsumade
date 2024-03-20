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
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;

public class Logger {
    private static Logger instance;
    private BooleanLogEntry intakeSensorEntry, hopperBottomSensorEntry, hopperTopSensorEntry;
    private DoubleLogEntry gyroAngleEntry, drivetrainSpeedEntry, intakeCurrentEntry, intakeSpeedEntry, hopperCurrentEntry, 
                leftFlywheelCurrentEntry,rightFlywheelCurrentEntry,armAngleEntry,leftFlywheelRPMEntry,rightFlywheelRPMEntry,
                LLDistanceEntry, armCurrentEntry, armAngleSetpointEntry, climberLeftArmPosition, climberRightArmPosition,
                climberLeftArmCurrent, climberRightArmCurrent, numOfApriltagEntry, stdDevEntry;;
    private StringLogEntry robotStateEntry, commandEntry;
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
    private Hopper hopper;
    private Superstructure superstructure;

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
        hopper = Hopper.getInstance();
        superstructure = Superstructure.getInstance();
        climber = Climber.getInstance();

        // Superstructure Logs
        robotStateEntry = new StringLogEntry(log, "/Superstructure/Robot State");

        // Drivetrain Logs
        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        drivetrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        fieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Position");
        botposeFieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Botpose position");
        moduleSpeedsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Positions");
        stdDevEntry = new DoubleLogEntry(log, "/Drivetrain/Megatag stddev");

        // Intake Logs
        intakeSensorEntry = new BooleanLogEntry(log, "/Intake/Intake Sensor");
        intakeCurrentEntry = new DoubleLogEntry(log, "/Intake/Intake Current");
        intakeSpeedEntry = new DoubleLogEntry(log, "/Intake/Intake Speed");

        // Hopper Logs
        hopperBottomSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Bottom Sensor");
        hopperTopSensorEntry = new BooleanLogEntry(log, "/Hopper/Hopper Top Sensor");
        hopperCurrentEntry = new DoubleLogEntry(log, "/Hopper/Hopper Current");

        //flywheel Loges
        rightFlywheelCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Right Motor Current");
        leftFlywheelCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Left Motor Current");

        rightFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Right RPM");
        leftFlywheelRPMEntry = new DoubleLogEntry(log, "/Flywheel/Left RPM");

        //arm logs
        armAngleEntry = new DoubleLogEntry(log, "/Arm/Angle Degrees");
        armCurrentEntry = new DoubleLogEntry(log, "/Arm/Current");
        armAngleSetpointEntry = new DoubleLogEntry(log, "/Arm/Angle Setpoint");

        //climber logs
        climberLeftArmPosition = new DoubleLogEntry(log, "/Climber/Left Arm Motor Position");
        climberRightArmPosition = new DoubleLogEntry(log, "/Climber/Right Arm Motor Position");
        climberLeftArmCurrent = new DoubleLogEntry(log, "/Climber/Left Arm Motor Current");
        climberRightArmCurrent = new DoubleLogEntry(log, "/Climber/Right Arm Motor Current");

        //LL logs
        LLDistanceEntry = new DoubleLogEntry(log, "/Limelight/Distance");
        numOfApriltagEntry = new DoubleLogEntry(log, "/Limelight/Number of Apriltags");

        //commands used
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, Boolean isStart) {
        commandEntry.append(event + (isStart? " Started": " Ended"));

    }

    public void updateLogs() {
        // Superstructure
        robotStateEntry.append(superstructure.getRobotState());

        // Drivetrain
        updateDrivetrainLogs();

        // Intake
        intakeSensorEntry.append(intake.getSensor());
        intakeCurrentEntry.append(intake.getMotorCurrent());
        intakeSpeedEntry.append(intake.getIntakeSpeed());

        // Hopper
        hopperBottomSensorEntry.append(hopper.getBottomSensor());
        hopperTopSensorEntry.append(hopper.getTopSensor());
        hopperCurrentEntry.append(hopper.getMotorCurrent());

        //Flywheel
        rightFlywheelCurrentEntry.append(flywheel.getRightMotorCurrent());
        leftFlywheelCurrentEntry.append(flywheel.getLeftMotorCurrent());

        rightFlywheelRPMEntry.append(flywheel.getFlywheelRightRPM());
        leftFlywheelRPMEntry.append(flywheel.getFlywheelLeftRPM());

        //Arm
        armAngleEntry.append(arm.getArmAngleDegrees());
        armCurrentEntry.append(arm.getSupplyCurrent());
        armAngleSetpointEntry.append(arm.getArmAngleSetpoint());

        //limelight
        LLDistanceEntry.append(limelightShooter.getDistance());
        numOfApriltagEntry.append(drivetrain.getNumApriltags());

        //Climber
        climberLeftArmCurrent.append(climber.getLeftArmCurrent());
        climberRightArmCurrent.append(climber.getRightArmCurrent());

        climberLeftArmPosition.append(climber.getLeftArmPosition());
        climberRightArmPosition.append(climber.getRightArmPosition());
    }

    //put pose to log for field visualization
    //put swerve module speeds and positions to log
    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(drivetrain.getHeading());
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

        SmartDashboard.putNumber("Standard Deviation from Logger", drivetrain.getStandardDeviation());
    }

    public void signalRobotEnable() {
        lastTeleopEnable = Timer.getFPGATimestamp();
    }
}
