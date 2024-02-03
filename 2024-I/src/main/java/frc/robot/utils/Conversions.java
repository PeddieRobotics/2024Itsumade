package frc.robot.utils;

public class Conversions {

    public static double falconToMeters(double falconTicks, double wheelDiameter, double gearRatio){
        return (Double)((Double)(Math.PI * falconTicks * wheelDiameter) / (Double)gearRatio);
    }
    
}
