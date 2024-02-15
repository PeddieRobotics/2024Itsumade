package frc.robot.subsystems;

//import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; -- ONLY using this for arm angle currently
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.FlywheelConstants;
import frc.robot.utils.Constants.ScoringConstants;

public class Flywheel {

    private static Flywheel instance;
    private double leftSetpoint, rightSetpoint;

    private Kraken flywheelLeftMotor, flywheelRightMotor;
    // private LaserCan flywheelSensor;
    private double rpmDelta;


    public Flywheel() {
        rpmDelta = SmartDashboard.getNumber("Flywheel Delta", 0.0);

        flywheelLeftMotor = new Kraken(RobotMap.FLYWHEEL_LEFT_MOTOR, RobotMap.CANIVORE_NAME);
        flywheelRightMotor = new Kraken(RobotMap.FLYWHEEL_RIGHT_MOTOR, RobotMap.CANIVORE_NAME);
        //flywheelSensor = new LaserCan(RobotMap.FLYWHEEL_SENSOR_ID);

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

    public void runFlywheelVelocitySetpoint(double speed){
        flywheelLeftMotor.setVelocityWithFeedForward(speed);
        flywheelRightMotor.setVelocityWithFeedForward(speed);
        leftSetpoint = speed;
        rightSetpoint = speed;
    }

    public void runLeftFlywheelVelocitySetpoint(double speed){
        flywheelLeftMotor.setVelocityWithFeedForward(speed);
        leftSetpoint = speed;
    }

    public void runRightFlywheelVelocitySetpoint(double speed){
        flywheelRightMotor.setVelocityWithFeedForward(speed);
        rightSetpoint = speed;
    }

    public void runFlywheelAmp(){
        runFlywheelVelocitySetpoint(ScoringConstants.kFlywheelAmpRPM);
    }

    public void runFlywheelHP(){
        runFlywheelVelocitySetpoint(ScoringConstants.kFlywheelHPIntakeRPM);
    }

    public void runFlywheelShot(){
        runFlywheelVelocitySetpoint(ScoringConstants.kFlywheelLayupRPM + rpmDelta); //for now, we're assumign you're using the same velocity for both LL and Layup
    }

    public double getFlywheelLeftRPM(){
        return flywheelLeftMotor.getRPM();
    }

    public double getFlywheelRightRPM(){
        return flywheelRightMotor.getRPM();
    }

    public void putSmartDashboard(){
        SmartDashboard.putBoolean("Update Flywheel PID", false);
        SmartDashboard.putNumber("Flywheel Left Motor RPM Setpoint", leftSetpoint);
        SmartDashboard.putNumber("Flywheel Right Motor RPM Setpoint", rightSetpoint);

        SmartDashboard.putNumber("Flywheel S", FlywheelConstants.kFlywheelS);
        SmartDashboard.putNumber("Flywheel V", FlywheelConstants.kFlywheelV);
        SmartDashboard.putNumber("Flywheel A", FlywheelConstants.kFlywheelA);
        SmartDashboard.putNumber("Flywheel P", FlywheelConstants.kFlywheelP);
        SmartDashboard.putNumber("Flywheel I", FlywheelConstants.kFlywheelI);
        SmartDashboard.putNumber("Flywheel D", FlywheelConstants.kFlywheelD);
        SmartDashboard.putNumber("Flywheel FF", FlywheelConstants.kFlywheelFF);
    }

    public void updateSmartDashboard() {
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

            flywheelLeftMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("Flywheel Left Motor RPM Setpoint", leftSetpoint));
            flywheelRightMotor.setVelocityWithFeedForward(SmartDashboard.getNumber("Flywheel Right Motor RPM Setpoint", rightSetpoint));
        }
    }

    public void periodic() {

    }

    private boolean isLeftAtRPM(){
        return (Math.abs(getFlywheelLeftRPM() - leftSetpoint) < ScoringConstants.kFlywheelShotThreshold);
    }

    private boolean isRightAtRPM(){
        return (Math.abs(getFlywheelRightRPM() - rightSetpoint) < ScoringConstants.kFlywheelShotThreshold);
    }

    public boolean isAtRPM() {
        return (isLeftAtRPM() && isRightAtRPM());
    }

}
