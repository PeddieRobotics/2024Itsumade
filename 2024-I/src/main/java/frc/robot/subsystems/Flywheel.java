package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.FlywheelConstants;
import frc.robot.utils.Constants.IntakeConstants;

public class Flywheel {

    private static Flywheel instance;
    private double flywheelSetpoint = 0;

    private Kraken flywheelLeftMotor, flywheelRightMotor;
    private LaserCan flywheelSensor;

    private InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

    public Flywheel() {
        flywheelLeftMotor = new Kraken(RobotMap.FLYWHEEL_LEFT_MOTOR, RobotMap.CANIVORE_NAME);
        flywheelRightMotor = new Kraken(RobotMap.FLYWHEEL_RIGHT_MOTOR, RobotMap.CANIVORE_NAME);
        flywheelSensor = new LaserCan(RobotMap.FLYWHEEL_SENSOR_ID);

        flywheelLeftMotor.setCurrentLimit(FlywheelConstants.kFlywheelLeftCurrentLimit);
        flywheelRightMotor.setCurrentLimit(FlywheelConstants.kFlywheelRightCurrentLimit);

        flywheelLeftMotor.setVelocityConversionFactor(FlywheelConstants.kFlywheelGearReduction);
        flywheelRightMotor.setVelocityConversionFactor(FlywheelConstants.kFlywheelGearReduction);

        flywheelLeftMotor.setVelocityPIDValues(
                FlywheelConstants.kFlywheelS,
                FlywheelConstants.kFlywheelV,
                FlywheelConstants.kFlywheelA,
                FlywheelConstants.kFlywheelP,
                FlywheelConstants.kFlywheelI,
                FlywheelConstants.kFlywheelD,
                FlywheelConstants.kFlywheelFF);

        flywheelRightMotor.setVelocityPIDValues(
                FlywheelConstants.kFlywheelS,
                FlywheelConstants.kFlywheelV,
                FlywheelConstants.kFlywheelA,
                FlywheelConstants.kFlywheelP,
                FlywheelConstants.kFlywheelI,
                FlywheelConstants.kFlywheelD,
                FlywheelConstants.kFlywheelFF);

        for(double[] pair:Constants.ScoringConstants.treeMapValues){
            map.put(pair[0],pair[1]);
        }
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    public void runFlywheelPercentOutput(double speed) {
        flywheelLeftMotor.setMotor(speed);
        flywheelRightMotor.setMotor(speed);
    }

    public void runLeftFlywheelPercentOutput(double speed) {
        flywheelLeftMotor.setMotor(speed);
    }

    public void runRightFlywheelPercentOutput(double speed) {
        flywheelRightMotor.setMotor(speed);
    }

    public void stopFlywheel() {
        flywheelLeftMotor.setMotor(0);
        flywheelRightMotor.setMotor(0);
    }

    public boolean getSensorReading(){
        if(getSensorMeasurement() < FlywheelConstants.kFlywheelSensorThreshold){
            return true;
        }
        else return false;
    }

    public double getSensorMeasurement(){
        return flywheelSensor.getMeasurement().distance_mm;
    }

    public void runFlywheelVelocitySetpoint(double speed){
        flywheelLeftMotor.setVelocityWithFeedForward(speed);
        flywheelRightMotor.setVelocityWithFeedForward(speed);
    }

    public void runLeftFlywheelVelocitySetpoint(double speed){
        flywheelLeftMotor.setVelocityWithFeedForward(speed);
    }

    public void runRightFlywheelVelocitySetpoint(double speed){
        flywheelRightMotor.setVelocityWithFeedForward(speed);
    }

    public void llshoot(){
        double llDist = 0; // temp value, change later to actually get value
        runFlywheelVelocitySetpoint(map.get(llDist));
    }

    public void layup(){
        runFlywheelVelocitySetpoint(Constants.ScoringConstants.kLayupVelocity);
    }

    public void putSmartDashboard(){
        SmartDashboard.putBoolean("Update Flywheel PID", false);
        SmartDashboard.putNumber("Flywheel Left Motor RPM Setpoint", flywheelSetpoint);
        SmartDashboard.putNumber("Flywheel Right Motor RPM Setpoint", flywheelSetpoint);

        SmartDashboard.putNumber("Flywheel S", FlywheelConstants.kFlywheelS);
        SmartDashboard.putNumber("Flywheel V", FlywheelConstants.kFlywheelV);
        SmartDashboard.putNumber("Flywheel A", FlywheelConstants.kFlywheelA);
        SmartDashboard.putNumber("Flywheel P", FlywheelConstants.kFlywheelP);
        SmartDashboard.putNumber("Flywheel I", FlywheelConstants.kFlywheelI);
        SmartDashboard.putNumber("Flywheel D", FlywheelConstants.kFlywheelD);
        SmartDashboard.putNumber("Flywheel FF", FlywheelConstants.kFlywheelFF);
    }

    public void updateSmartdashBoard() {
        if(SmartDashboard.getBoolean("Update Flywheel PID", false)){
            flywheelLeftMotor.setVelocityPIDValues(
                SmartDashboard.getNumber("Flywheel S", FlywheelConstants.kFlywheelS), 
                SmartDashboard.getNumber("Flywheel V", FlywheelConstants.kFlywheelV), 
                SmartDashboard.getNumber("Flywheel A", FlywheelConstants.kFlywheelA), 
                SmartDashboard.getNumber("Flywheel P", FlywheelConstants.kFlywheelP), 
                SmartDashboard.getNumber("Flywheel I", FlywheelConstants.kFlywheelI), 
                SmartDashboard.getNumber("Flywheel D", FlywheelConstants.kFlywheelD), 
                SmartDashboard.getNumber("Flywheel FF", FlywheelConstants.kFlywheelFF)
            );

            flywheelRightMotor.setVelocityPIDValues(
                SmartDashboard.getNumber("Flywheel S", FlywheelConstants.kFlywheelS), 
                SmartDashboard.getNumber("Flywheel V", FlywheelConstants.kFlywheelV), 
                SmartDashboard.getNumber("Flywheel A", FlywheelConstants.kFlywheelA), 
                SmartDashboard.getNumber("Flywheel P", FlywheelConstants.kFlywheelP), 
                SmartDashboard.getNumber("Flywheel I", FlywheelConstants.kFlywheelI), 
                SmartDashboard.getNumber("Flywheel D", FlywheelConstants.kFlywheelD), 
                SmartDashboard.getNumber("Flywheel FF", FlywheelConstants.kFlywheelFF)
            );

            flywheelLeftMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("Flywheel Left Motor RPM Setpoint", flywheelSetpoint));
            flywheelRightMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("Flywheel Right Motor RPM Setpoint", flywheelSetpoint));
        }
    }

    public void periodic() {

    }

}
