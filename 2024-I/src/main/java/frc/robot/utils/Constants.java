package frc.robot.utils;

public final class Constants {
  public static class GlobalConstants {
    public static final int kDriverControllerPort = 0;
    public static final boolean useLEDLights = true;
  }

  public static class OIConstants {
    public static final boolean useDebugModeLayout = true;
    public static final double drivingDeadband = 0.1;
  }

  public static class DriveConstants {
    public static final int kDriveCurrentLimit = 30;
    public static final int kTurningCurrentLimit = 30;

    public static final double kDrivingMotorGearRatio = 0;
    public static final double kWheelRadius = 2.0;
  }

  public static class ModuleConstants {
    public static final double kDrivingMotorCurrentLimit = 50;
    public static final double kTurningMotorCurrentLimit = 50;

    public static final double kDrivingP = 0.0;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;

    public static final double kTurningP = 0.0;
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
}
