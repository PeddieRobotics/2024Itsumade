package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Arm;

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
        swerveModuleLocations[3]
    );

    // TODO: Change this value
    public static final double kMaxFloorSpeed = 4.5; // meters per second
    public static final double kMaxAngularSpeed = (3.0 / 2.0) * Math.PI; // radians per second
    // public static final double kMaxFloorSpeed = 4.0; // meters per second
    // public static final double kMaxAngularSpeed = Math.PI; // radians per second

    public static final int kDriveCurrentLimit = 30;
    public static final int kTurningCurrentLimit = 20;

    public static final double kWheelRadius = 2.0;

    // Steps to doing offsets for swerves:
    // 1. remove offsets (set all to 0) and then deploy code
    // 2. spin all modules so that bevel gears face left relative to robot (shooter
    // in front)
    // 3. read the cancoder values from dashboard, and put those values for these
    // offsets (check robotmap for ids)
    public static final double kFrontLeftModuleAngularOffset = -2.896;
    public static final double kFrontRightModuleAngularOffset = -3.073;
    public static final double kBackLeftModuleAngularOffset = -2.045;
    public static final double kBackRightModulelAngularOffset = -1.978;

    public static final double kHeadingCorrectionP = 0.1;
    public static final double kHeadingCorrectionTolerance = 1.0;
  }

  public static class ModuleConstants {
    public static final double kWheelDiameterInches = 3.6;

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

    // designated shooting coordinates (for ToClosestShooting command)
    // blue coordinates (x, y, theta)
    public static final double[][] shootingPositions = {
        { 3.95, 6.88, -160.00 },
        { 3.81, 5.31, 175.00 },
        { 2.21, 4.47, 125.00 },
        { 3.19, 2.68, 135.00 }
    };

    public static final double kFollowNoteEarlyEndMinDuration = 0.10;
    public static final double kFollowNoteEarlyEndMaxDuration = 0.25;
    public static final double kFollowNoteNoNotePercent = 0.80;
    public static final double kFollowNoteNotSameNoteThresh = 2.5;
    public static final double kFollowNoteSpeed = 1.3;

    public static final double kLimelightPrepDeadlineTime = 0.7;
    public static final double kLayupPrepDeadlineTime = 0.7;
    public static final double kScoreDeadlineTime = 1.0;
    public static final double kTargetDeadlineTime = 3.0;
  }

  public static class LimelightConstants {
    public static final double kLimelightHeight = 17.88;
    public static final double kLimelightPanningAngle = 25;
    public static final double kLimelightAngle = 0;

    // drive to target command constants
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

    // follow note command constants
    public static final double kFollowNoteTurnP = 0.05;
    public static final double kFollowNoteTurnI = 0;
    public static final double kFollowNoteTurnD = 0;
    public static final double kFollowNoteTurnFF = 0;
    public static final double kFollowNoteAngleThreshold = 1;

    // target apriltag command constants
    public static final double kTargetP = 0.017;
    public static final double kTargetI = 0;
    public static final double kTargetD = 0.002;
    public static final double kTargetFF = 0.01;

    public static final double kTargetAngleThreshold = 1.0;

    // speaker april tag height because that is what will be used the most
    public static final double kSpeakerAprilTagHeight = 57.125;

    public static final int kShooterAprilTagPipeline = 3;
    public static final int kShooterTargetingPipeline = 4;

    public static final int kIntakeNotePipeline = 0;
  }

  public static class ArmConstants {
    public static final int kArmPrimaryCurrentLimit = 40;

    public static final double kArmS = 4;
    public static final double kArmV = 0;
    public static final double kArmA = 6;
    public static final double kArmP = 700;
    public static final double kArmI = 0;
    public static final double kArmD = 30;
    public static final double kArmFF = 0;
    public static final double kArmIZone = 0;
    public static final double kArmG = 8.5;

    public static final double kCancoderCruiseVelocityRPS = .5;
    public static final double kCancoderCruiseMaxAccel = 1; // rot/s^2
    public static final double kCancoderCruiseMaxJerk = 10; // rot/s^3

    public static final double kArmForwardSoftLimit = 0.3; // in mechanism rotations
    public static final double kArmReverseSoftLimit = -0.1; // in mechanism rotations

    public static final double kArmMagnetOffset = 0.237315444; // see spreadsheet "FIRST Calculations" for reference

    // TOTAL NET gear reduction from motor ALL THE WAY to shoulder pivot
    public static final double kRotorToArmGearReduction = 16384.0 / 125;

    // gear reduction from motor to CANCoder Shaft
    public static final double kRotorToSensorRatio = (kRotorToArmGearReduction / 2);
    public static final double kArmSensorToMechanismRatio = 2.0; // dependednt on feedback device
    // hypothetical values for the arm when going to these various positions

    public static final double kArmIntakeHPPosition = 70; // in deg
    public static final double kArmPositionEpsilon = 3; // temporarily loose due to arm tuning, can tighten this later
                                                        // when code improves
    public static final double kArmAmpPosition = 120;
    public static final double kArmFrontLayupPosition = 38; // 38
    public static final double kArmSideLayupPosition = 35;
    public static final double kArmStowPosition = 0;
    public static final double kArmIntakePositionFromGround = 32;

  }

  public static class ClimberConstants {
    public static final double kClimberCurrentLimit = 60;
    public static final double kClimberForwardSoftLimit = 150;
    public static final double kClimberReverseSoftLimit = 0;

    public static final double kClimberP = 0.0;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;
    public static final double kClimberFF = 0.0;
    public static final double kClimberRetractPercentOutput = -0.7; // percent putput the climber would
    public static final double kClimberDeployPercentOutput = 0.7; // percent putput the climber would
    public static final double kClimberDeployPosition = 140.0;
    public static final double kClimberRetractPosition = 30.0;
    public static final double kClimberAngleTolerance = 2.0;

    public static final double kClimberGearReduction = 35.0 / 1.0;

    public static final int CLIMBER_SENSOR_ID = 0;
  }

  public static class FlywheelConstants {
    public static final int kFlywheelLeftCurrentLimit = 30;
    public static final int kFlywheelRightCurrentLimit = 30;

    public static final int kFlywheelForwardTorqueCurrentLimit = 40;
    public static final int kFlywheelReverseTorqueCurrentLimit = 0;

    public static final double kFlywheelGearReduction = 30.0 / 24.0;

    public static final double kFlywheelSensorThreshold = 0;

    public static final double kFlywheelS = 3.5;
    public static final double kFlywheelV = 0; // 0.00225;
    public static final double kFlywheelA = 0;
    public static final double kFlywheelP = 5; // could be changed later... crap tune
    public static final double kFlywheelI = 5;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelFF = 0;
  }

  public static class ScoringConstants {
    public static final double kLeftFlywheelLLShootingRPM = 4300;
    public static final double kRightFlywheelLLShootingRPM = 3200;
    public static final double kLeftFlywheelLayupRPM = 3500;
    public static final double kRightFlywheelLayupRPM = 2500;
    public static final double kLeftFlywheelAmpRPM = 500;
    public static final double kRightFlywheelAmpRPM = 500;
    public static final double kLeftFlywheelHPIntakeRPM = -750;
    public static final double kRightFlywheelHPIntakeRPM = -750;
    public static final double kFlywheelShotThreshold = 100;
    public static final double kShootingStateTime = 0.5;

        // Distance (horizontal inches to goal as estimated by LL), Angle (degrees) -
    // needs more tuning/initial values only
    public static final double[][] treeMapValues = new double[][] { { 44.0, 42.0 }, { 47.5, 43 }, { 60, 50 }, { 65.0, 53.0 }, { 70.1, 55.0 }, { 80, 56 }, { 85.2, 60.0 },
        { 93.8, 61 }, { 108.2, 65.6 }, { 112.5, 66 }, { 130, 68 }, { 134.6, 69.0 }, { 147.8, 69.5 }, { 154.5, 70.5 }, { 158.6, 70.5 }, { 166.2, 71.5}, { 175.6, 72 }, { 179.6, 72.3 },
        { 213, 73.25 } };
  }

  public static class IntakeConstants {
    public static final int kIntakeCurrentLimit = 20;
    public static final int kHopperCurrentLimit = 20;
    public static final double kIntakeSensorThreshold = 0;
    public static final double kHopperSensorThreshold = 0;

    public static final double kIntakeSpeed = 0.85;
    public static final double kReverseIntakeSpeed = -0.85;
  }

  public static class HopperConstants {
    public static final double kHPIntakeHopperSpeed = 0; // if used, needs to be negative
    public static final double kGroundIntakeHopperSpeed = 0.5;
    public static final double kOuttakeHopperSpeed = -0.5;
    public static final double kFeedFlywheelAmpSpeed = 0.5;
    public static final double kFeedFlywheelLayupSpeed = 0.5;
    public static final double kFeedFlywheelSpeakerSpeed = 0.5;

  }

}