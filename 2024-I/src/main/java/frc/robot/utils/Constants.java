package frc.robot.utils;

public final class Constants {
    
    public static class GlobalConstants{
        public static final String canivore_name = "canivore";

    }

   public static class ArmConstants{
        public static int armCancoderID = 52;
        public static int armMotorID = 50;
        public static double armMagnetOffset = 0.11280345;
        public static int armCurrentLimit = 40;
        public static int armTorqueLimit = 40;
        public static double armRotorToGearReduction = 16384.0/125;
        public static double armRotorToSensorRatio = (armRotorToGearReduction)/2;
        public static double armSensorToMechanismRatio = 2;
        public static double armForwardSoftLimit = 0.3;
        public static double armReverseSoftLimit = -0.1;
        
   }

   public static class IntakeConstants{
    public static final int intakeSensorID = 9;
   }

    public static class FlywheelConstants{
        public static final int flywheel_orange = 8;
        public static final int flywheel_black = 7;

        public static final double flywheel_current_limit = 30;
        public static final double flywheel_torque_limit = 80;
    }

    public static class CartridgeConstants{
        public static final int cartridge_id = 6;
        public static final int cartridge_current_limit = 20;
        public static final int cartridge_lower_sensor = 8;
        public static final int cartridge_upper_sensor = 7;
    }
}
