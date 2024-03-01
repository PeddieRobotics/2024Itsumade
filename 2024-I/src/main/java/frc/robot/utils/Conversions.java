package frc.robot.utils;

public class Conversions {
    
    // Converts requested arm degrees with 0 straight down
    // into mechanism rotations starting at -0.1
    public static double convertArmDegreesToRotations(double degrees){
        return (degrees-37.0)/360.0;
    }

    public static double convertRotationsToArmDegrees(double rotations){
        return 360.0 * rotations + 37.0;
    }

}
