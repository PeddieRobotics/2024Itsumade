package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class GlobalConstants {
    public static final int kDriverControllerPort = 0;
    public static final boolean useLEDLights = true;
  }

  public static class OIConstants {
    public static final boolean useDebugModeLayout = true;
    public static final double kDrivingDeadband = 0.1;
  }

  public static class DriveConstants {
    // TODO: Update these constants
    public static final double kTrackWidth = Units.inchesToMeters(25.75);
    public static final double kWheelBase = Units.inchesToMeters(22.75);

    public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

    public static final Translation2d[] swerveModuleLocations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveModuleLocations[0],
        swerveModuleLocations[1],
        swerveModuleLocations[2],
        swerveModuleLocations[3]);

    // TODO: Change this value
    public static final double kMaxFloorSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // radians per second
    

    public static final int kDriveCurrentLimit = 30;
    public static final int kTurningCurrentLimit = 30;

    public static final double kWheelRadius = 2.0;

    public static final double kFrontLeftModuleAngularOffset = 2.88 + 0.59;
    public static final double kFrontRightModuleAngularOffset = -2.01 + 0.197 - 2.68;
    public static final double kBackLeftModuleAngularOffset = 2.09 + 2.08;
    public static final double kBackRightModulelAngularOffset = -2.26 - 1.75;

    
    public static final double kHeadingCorrectionP = 0.1;
    public static final double kHeadingCorrectionTolerance = 1.0;
  }

  public static class ModuleConstants {
    public static final double kWheelDiameterInches = 4.0;

    public static final double kDriveMotorReduction = 6.12;
    public static final double kDrivingEncoderPostionFactor = (Units.inchesToMeters(kWheelDiameterInches) * Math.PI)
        / kDriveMotorReduction;
    public static final double kDrivingEncoderVelocityFactor = (Units.inchesToMeters(kWheelDiameterInches) * Math.PI
        / kDriveMotorReduction);

    public static final double kTurningMotorReduction = 150.0 / 7.0;
    public static final double kTurningEncoderPositonFactor = kTurningMotorReduction / (2 * Math.PI);
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60;

    public static final double kDrivingMotorCurrentLimit = 50;
    public static final double kTurningMotorCurrentLimit = 50;

    public static final double kDrivingS = 0.05;
    public static final double kDrivingV = 0.13;
    public static final double kDrivingA = 0.0;
    public static final double kDrivingP = 0.11;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;

    public static final double kTurningP = 36;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;

  }

  public static class AutoConstants {
    public static final double kTranslationP = 5.0;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    public static final double kThetaP = 5.0;
    public static final double kThetaI = 0.0;
    public static final double kThetaD = 0.0;
  }

  public static class LimelightConstants {
    public static final double kLimelightHeight = 17;
    public static final double kLimelightPanningAngle = 60;
    public static final double kLimelightAngle = 0;

    //drive to target command constants
    public static final double kDriveToTargetTurnP = 0;
    public static final double kDriveToTargetTurnI = 0;
    public static final double kDriveToTargetTurnD = 0;
    public static final double kDriveToTargetIZone = 0;
    public static final double kDriveToTargetTurnFF = 0;
    public static final double kDriveToTargetTurnThreshold = 0;

    public static final double kDriveToTargetMoveP = 0;
    public static final double kDriveToTargetMoveI = 0;
    public static final double kDriveToTargetMoveD = 0;
    public static final double kDriveToTargetMoveFF = 0;
    public static final double kDriveToTargetMoveThreshhold = 0;

    //follow note command constants
    public static final double kFollowNoteTurnP = 0;
    public static final double kFollowNoteTurnI = 0;
    public static final double kFollowNoteTurnD = 0;
    public static final double kFollowNoteAngleThreshold = 0;

    //target apriltag command constants
    public static final double kTargetP = 0.03;
    public static final double kTargetI = 0.1;
    public static final double kTargetD = 0;
    public static final double kTargetFF = 0;

    public static final double kTargetAngleThreshold = 2.0;
  }

  public static class ArmConstants {
    public static final int kArmPrimaryCurrentLimit = 40;
    public static final double kArmS = 0.25;
    public static final double kArmV = 0;
    public static final double kArmA = 0;
    public static final double kArmP = 40;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmFF = 0;
    public static final double kArmIZone = 0;
    public static final double kArmForwardSoftLimitDegrees = 100;
    public static final double kArmReverseSoftLimitDegrees = -36;
    public static final double kArmGearReduction = 0;
    public static final double kArmPositionConversionFactor = 1.0; // dependednt on feedbakc device
    //hypothetical values for the arm when going to these various positions
    public static final double kArmIntakeHPPosition = 0.0; // in deg
    public static final double kArmPositionEpsilon = 1;
    public static final double kArmAmpPosition = 45.0;
    public static final double kArmLayupPosition = -80.0;
    public static final double kArmStowPosition = -90.0;
    public static final double kArmIntakePositionFromGround = -70.0;
    public static final double kArmPositionOffsetDegrees = 58.95-125.8; //hehe

    public static final double cancoderCruiseVelocityRPS=.3;
    public static final double cancoderCruiseMaxAccel=200; // rot/s^2
    public static final double cancoderCruiseMaxJerk=1600; //rot/s^3

    public static final double kRotorToSensorRatio = 16384.0/125;
  }

  public static class ClimberConstants {
    public static final double kClimberRightCurrentLimit = 0;
    public static final double kClimberLeftCurrentLimit = 0;
    public static final double kClimberP = 0.0;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;
    public static final double kClimberFF = 0.0;
    public static final double kClimberPercentOutput = -0.7; //percent putput the climber would 
    public static final double kClimberUnwindPosition = 0.0; //the angle the kraken needs to go backwards to retract back

    public static final double kClimberGearReduction = 35.0 / 1.0;

    public static final int CLIMBER_SENSOR_ID = 0;
  }

  public static class FlywheelConstants {
    public static final int kFlywheelLeftCurrentLimit = 0;
    public static final int kFlywheelRightCurrentLimit = 0;

    public static final double kFlywheelGearReduction = 30.0 / 18.0;

    public static final double kFlywheelSensorThreshold = 0;

    public static final double kFlywheelS = 0;
    public static final double kFlywheelV = 0;
    public static final double kFlywheelA = 0;
    public static final double kFlywheelP = 0;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelFF = 0;
  }

  public static class ScoringConstants {
    public static final double kFlywheelLLShootingRPM = 4000;
    public static final double kFlywheelLayupRPM = 2500;
    public static final double kFlywheelAmpRPM = 500;
    public static final double kFlywheelHPIntakeRPM = -250;
    public static final double kFlywheelShotThreshold = 100; 
    // Distance, Angle; SAMPLE VALUES, NEEDS TO BE TUNED -TONY
    public static final double[][] treeMapValues = new double[][] { { 0.5, -55 }, { 1.5, -45 }, {2.5, -37.5}, {3.5, -25}, {4.5, -20}, {5.5, -15} }; 
  }

  public static class IntakeConstants {
    public static final int kIntakeCurrentLimit = 20;
    public static final int kHopperCurrentLimit = 0;
    public static final double kIntakeSensorThreshold = 0;
    public static final double kHopperSensorThreshold = 0;

    public static final double kIntakeSpeed = 0.75;
  }

  public static class HopperConstants {
    public static final double kHopperSpeed = 0;
    public static final double kFeedSpeed = 0;
  }

}