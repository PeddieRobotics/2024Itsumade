package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double kTrackWidth = Units.inchesToMeters(20);
    public static final double kWheelBase = Units.inchesToMeters(20);

    public static final double kBaseRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

    public static final Translation2d[] swerveModuleLocations = {
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0)
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
  }

  public static class ModuleConstants {
    public static final double WHEEL_DIAMETER_IN = 4.0;

    public static final double kDriveMotorReduction = 6.12;
    public static final double kDrivingEncoderPostionFactor = (Units.inchesToMeters(WHEEL_DIAMETER_IN) * Math.PI)
        / kDriveMotorReduction;
    public static final double kDrivingEncoderVelocityFactor = (Units.inchesToMeters(WHEEL_DIAMETER_IN) * Math.PI
        / kDriveMotorReduction);

    public static final double kTurningMotorReduction = 150.0 / 7.0;
    public static final double kTurningEncoderPositonFactor = 2 * Math.PI / kTurningMotorReduction;
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

    public static final double kTurningP = 1.75;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;
  }

  public static class AutoConstants {

  }

  public static class LimelightConstants {
    public static final double limelightHeight = 0;
    public static final double limelightPanningAngle = 0;
    public static final double limelightAngle = 0;
  }

  public static class ArmConstants {
    public static final int ARM_PRIMARY_MOTOR_LIMIT = 0;
    public static final int ARM_SECONDARY_MOTOR_LIMIT = 0;
    public static final int ARM_P = 0;
    public static final int ARM_I = 0;
    public static final int ARM_D = 0;
    public static final int ARM_FF = 0;
    public static final int ARM_IZONE = 0;
  }

  public static class FlywheelConstants {
    public static final int FLYWHEEL_PRIMARY_MOTOR_LIMIT = 0;
    public static final int FLYWHEEL_SECONDARY_MOTOR_LIMIT = 0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_LIMIT = 0;
    public static final int HOPPER_MOTOR_LIIMT = 0;
  }

  public static class PIDConstants {
    public static final double VELOCITY_S = 0.05;
    public static final double VELOCITY_V = 0.12;
    public static final double VELOCITY_P = 0.08;
    public static final double VELOCITY_I = 0;
    public static final double VELOCITY_D = 0;
    public static final double VELOCITY_FF = 0;

    public static final double POSITION_P = 0;
    public static final double POSITION_I = 0;
    public static final double POSITION_D = 0;
    public static final double POSITION_FF = 0;
  }
}