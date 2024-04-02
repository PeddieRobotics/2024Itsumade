package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
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
    public static final double kMaxFloorSpeed = 5.0; // meters per second
    public static final double kMaxAngularSpeed = (3.0 / 2.0) * Math.PI; // radians per second

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

    public static final boolean kUseMegaTag = true; // Enable megatag botpose updates in teleop. Force calibrate works no matter what this is set to (in both auto and teleop).
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
    public static final double kLimelightHeight = 17.88; // height in inches
    public static final double kLimelightPanningAngle = 25; // mounting angle in degrees

    // drive to target command constants
    public static final double kDriveToTargetTurnP = 0.05;
    public static final double kDriveToTargetTurnI = 0.0002;
    public static final double kDriveToTargetTurnD = 0.0;
    public static final double kDriveToTargetIZone = 5;
    public static final double kDriveToTargetTurnFF = 0;
    public static final double kDriveToTargetTurnThreshold = 1.5;

    public static final double kDriveToTargetMoveP = 0.058;
    public static final double kDriveToTargetMoveI = 0.0025;
    public static final double kDriveToTargetMoveD = 0.005;
    public static final double kDriveToTargetMoveFF = 0.03;
    public static final double kDriveToTargetMoveThreshold = 1.5;

    // follow note command constants
    public static final double kFollowNoteTurnP = 0.05;
    public static final double kFollowNoteTurnI = 0;
    public static final double kFollowNoteTurnD = 0;
    public static final double kFollowNoteTurnFF = 0;
    public static final double kFollowNoteAngleThreshold = 1;

    // target speaker apriltag command (use tx from limelight) constants
    public static final double kTargetP = 0.026;
    public static final double kTargetI = 0.02;
    public static final double kTargetD = 0.0025;
    public static final double kTargetFF = 0.065;
    public static final double kTargetIZone = 1;
    public static final double kTargetTarget = 4;

    public static final double kTargetAutoP = 0.02;
    public static final double kTargetAutoI = 0.0015;
    public static final double kTargetAutoD = 0.002;
    public static final double kTargetAutoFF = 0.03;
    public static final double kTargetAutoIZone = 4.0;
    public static final double kTargetAutoTarget = 4;

    // target speaker apriltag (snap with gyro using one-time tx calculation) command constants
    public static final double kSnapToSpeakerP = 0.05;
    public static final double kSnapToSpeakerI = 0.01;
    public static final double kSnapToSpeakerD = 0.0;
    public static final double kSnapToSpeakerFF = 0.03;

    // general targeting using odometry (use current odometry to turn to an expected goal location) command constants
    public static final double kOdometryTargetP = 0.03;
    public static final double kOdometryTargetI = 0.001;
    public static final double kOdometryTargetD = 0.0;
    public static final double kOdometryTargetFF = 0.02;


    // align to amp AprilTag (horizontal)
    public static final double kHorizontalAlignP = 0.02;
    public static final double kHorizontalAlignI = 0.0;
    public static final double kHorizontalAlignD = 0.0;
    public static final double kHorizontalAlignFF = 0.0;

    // Speaker coordinates in meters using blue coordinate system
    // THESE NUMBERS MUST BE ADJUSTED IF FMAP IS ADJUSTED ON X or Y AXIS
    public static final double kRedSpeakerPositionX = 16.61795; // 2* 8.308975 (distance from midline to April Tag)
    public static final double kRedSpeakerPositionY = 5.548249; 
    
    public static final double kBlueSpeakerPositionX = 0.0;
    public static final double kBlueSpeakerPositionY = 5.548249; // 4.105656 (dist between field wall and midline) + 1.442593 (.fmap offset from midline to tag) 

    // Corner to target while passing using blue coordinate system
    public static final double kRedCornerPassingX = 15.65;
    public static final double kRedCornerPassingY = 6.7; 
    
    public static final double kBlueCornerPassingX = 0.85;
    public static final double kBlueCornerPassingY = 6.7;


    // center of speaker april tag height in inches (used for LL distance calculations)
    // THIS MUST BE ADJUSTED IF FMAP IS ADJUSTED ON Z AXIS
    public static final double kSpeakerAprilTagHeight = 57.125;
  
    // Threshold (in degrees) for convergence on all speaker targeting algorithms
    public static final double kTargetAngleThreshold = 0.2;
    
    //threshold (in degrees) for the amp align rotation
    public static final double kAmpAlignAngleThreshold = 1.0;

    //amp rotation -- degrees, this never changes really
    public static final double kAmpOdometryHeading = 90; //angle of amp relative to odometry

    public static final int kShooterPipeline = 0;
    public static final int kIntakeNotePipeline = 0;
  }

  public static class ArmConstants {
    public static final int kArmPrimaryCurrentLimit = 40;
    public static final int kArmForwardTorqueCurrentLimit = 40;
    public static final int kArmReverseTorqueCurrentLimit = -40;

    public static final double kArmS = 4;
    public static final double kArmV = 0;
    public static final double kArmA = 6;
    public static final double kArmP = 700;
    public static final double kArmI = 0;
    public static final double kArmD = 30;
    public static final double kArmFF = 0;
    public static final double kArmIZone = 0;
    public static final double kArmG = 8.5;

    public static final double kCancoderCruiseVelocityRPS = 0.7;
    public static final double kCancoderCruiseMaxAccel = 1.6; // rot/s^2
    public static final double kCancoderCruiseMaxJerk = 16; // rot/s^3

    public static final double kArmForwardSoftLimit = 0.3; // in mechanism rotations
    public static final double kArmReverseSoftLimit = -0.1; // in mechanism rotations

    public static final double kArmMagnetOffset = 0.11280345; // see spreadsheet "FIRST Calculations" or mechanisms sheet for reference

    // TOTAL NET gear reduction from motor ALL THE WAY to shoulder pivot
    public static final double kRotorToArmGearReduction = 16384.0 / 125;

    // gear reduction from motor to CANCoder Shaft
    public static final double kRotorToSensorRatio = (kRotorToArmGearReduction / 2);
    public static final double kArmSensorToMechanismRatio = 2.0; // dependent on feedback device
    // hypothetical values for the arm when going to these various positions

    public static final double kArmIntakeHPPosition = 70; // in deg
    public static final double kArmPositionEpsilon = 3; // in deg
    public static final double kArmStowPositionEpsilon = 10; // in deg
    public static final double kArmAmpPosition = 133; //124 in lab, 140 at comp 134 IS LAST USED ANGLE AT HATBORO
    public static final double kArmAmpPrepPosition = 110;
    public static final double kArmFrontLayupPosition = 38; // 38
    public static final double kArmSideLayupPosition = 35;
    public static final double kArmStowPosition = 0;
    public static final double kArmIntakePositionFromGround = 34;
    public static final double kArmPodiumShotPosition = 64; //was 62
    public static final double kArmLobPassPosition = 38;

    //multiplier to arm angle for lookuptable since real field is different from lab
    public static final double kArmLLDistMultiplier = 1.00; // 1.04; // 0.97; 0.98; 1.00; 1.03

  }

  public static class ClimberConstants {
    public static final double kClimberCurrentLimit = 60;
    public static final double kClimberForwardSoftLimit = 150;
    public static final double kClimberReverseSoftLimit = 0;

    public static final double kClimberP = 0.0;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;
    public static final double kClimberFF = 0.0;
    public static final double kClimberRetractPercentOutput = -1;
    public static final double kClimberDeployPercentOutput = 1;
    public static final double kClimberDeployPosition = 140.0;
    public static final double kClimberRetractPosition = 0.0;
    public static final double kClimberAngleTolerance = 2.0;

    public static final double kClimberGearReduction = 35.0 / 1.0;

    public static final int CLIMBER_SENSOR_ID = 0;
  }

  public static class FlywheelConstants {
    public static final int kFlywheelLeftCurrentLimit = 30;
    public static final int kFlywheelRightCurrentLimit = 30;

    public static final int kFlywheelForwardTorqueCurrentLimit = 80;
    public static final int kFlywheelReverseTorqueCurrentLimit = 0;

    public static final double kFlywheelGearReduction = 30.0 / 24.0;

    public static final double kFlywheelSensorThreshold = 0;

    public static final double kFlywheelS = 3.5;
    public static final double kFlywheelV = 0;
    public static final double kFlywheelA = 0;
    public static final double kFlywheelP = 5;
    public static final double kFlywheelI = 5;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelFF = 0;

    public static final double kFlywheelLobPassSpeedMultiplier = 1.0;
  }

  public static class ScoringConstants {
    public static final double kLeftFlywheelLLShootingRPM = 4300;
    public static final double kRightFlywheelLLShootingRPM = 3200;

    public static final double kLeftFlywheelLLShootingFastRPM = 5200;
    public static final double kRightFlywheelLLShootingFastRPM = 3900;

    public static final double kFastFlywheelMultiply = 1.25;
    public static final double kFastFlywheelLimit = 90.0;

    public static final double kLeftFlywheelLayupRPM = 3500;
    public static final double kRightFlywheelLayupRPM = 2500;
    public static final double kLeftFlywheelAmpRPM = 3000;
    public static final double kRightFlywheelAmpRPM = 2000;
    public static final double kLeftFlywheelLobPassRPM = 2680;
    public static final double kRightFlywheelLobPassRPM = 2000;
    public static final double kLeftFlywheelHPIntakeRPM = -750;
    public static final double kRightFlywheelHPIntakeRPM = -750;
    public static final double kFlywheelShotThreshold = 100;
    public static final double kShootingStateTime = 0.1;

    public static final double kRevUpLeftFlywheelRPM = 3000;
    public static final double kRevUpRightFlywheelRPM = 2000;

    public static final double kLobPassSpeedMultiplier = 1.06;

        // Distance (horizontal inches to goal as estimated by LL), Angle (degrees) -
    // needs more tuning/initial values only
    public static final double[][] treeMapValues = new double[][] {{ 44.0, 42.0 }, { 47.5, 43 }, { 60, 50 }, { 65.0, 53.0 }, { 70.1, 55.0 }, { 80, 56 }, { 85.2, 60.0 },
        { 93.8, 61 }, { 108.2, 65.6 }, { 112.5, 66 }, { 130, 68 }, { 134.6, 69.0 }, {140.6, 69.1}, {150.3, 70.0}, {160.5, 70.7}, {170.15, 71.6}, {180.1, 72.15}, {185.5, 72.35}};

    public static final double[][] treeMapValuesOld = new double[][] { { 44.0, 42.0 }, { 47.5, 43 }, { 60, 50 }, { 65.0, 53.0 }, { 70.1, 55.0 }, { 80, 56 }, { 85.2, 60.0 },
        { 93.8, 61 }, { 108.2, 65.6 }, { 112.5, 66 }, { 130, 68 }, { 134.6, 69.0 }, { 147.8, 69.5 }, { 154.5, 70.5 }, { 158.6, 70.5 }, { 166.2, 71.5}, { 175.6, 72 }, { 179.6, 72.3 },
        { 213, 73.0 } };
  }

  public static class IntakeConstants {
    public static final int kIntakeCurrentLimit = 20;
    public static final int kHopperCurrentLimit = 20;
    public static final double kIntakeSensorThreshold = 0;
    public static final double kHopperSensorThreshold = 0;

    public static final double kIntakeSpeed = 1.0;
    public static final double kIntakeFeedSpeed = 0.5;
    public static final double kReverseIntakeSpeed = -1.0;

    public static final double kIntakeOpenLoopRampRate = 0.25;
  }

  public static class HopperConstants {
    public static final double kHPIntakeHopperSpeed = 0; // if used, needs to be negative
    public static final double kGroundIntakeHopperSpeed = 0.5;
    public static final double kOuttakeHopperSpeed = -0.5;
    public static final double kFeedFlywheelAmpSpeed = 0.5;
    public static final double kFeedFlywheelLayupSpeed = 0.5;
    public static final double kFeedFlywheelSpeakerSpeed = 0.5;
    public static final double kFeedFlywheelPodiumSpeed = 0.5;
    public static final double kFeedFlywheelLobPassSpeed = 0.5; 
  }

}