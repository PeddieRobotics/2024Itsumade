package frc.robot.subsystems;

//import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; -- ONLY using this for arm angle currently
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.FlywheelConstants;
import frc.robot.utils.Constants.ScoringConstants;

public class Flywheel extends SubsystemBase {

    private static Flywheel instance;
    private double leftSetpoint, rightSetpoint;

    private Kraken flywheelLeftMotor, flywheelRightMotor;
    private double rpmDelta;

    public Flywheel() {
        rpmDelta = 0.0;

        flywheelLeftMotor = new Kraken(RobotMap.FLYWHEEL_LEFT_MOTOR, RobotMap.CANIVORE_NAME);
        flywheelRightMotor = new Kraken(RobotMap.FLYWHEEL_RIGHT_MOTOR, RobotMap.CANIVORE_NAME);

        flywheelLeftMotor.setSupplyCurrentLimit(FlywheelConstants.kFlywheelLeftCurrentLimit);
        flywheelRightMotor.setSupplyCurrentLimit(FlywheelConstants.kFlywheelRightCurrentLimit);

        flywheelLeftMotor.setForwardTorqueCurrentLimit(FlywheelConstants.kFlywheelForwardTorqueCurrentLimit);
        flywheelLeftMotor.setReverseTorqueCurrentLimit(FlywheelConstants.kFlywheelReverseTorqueCurrentLimit);
        flywheelRightMotor.setForwardTorqueCurrentLimit(FlywheelConstants.kFlywheelForwardTorqueCurrentLimit);
        flywheelRightMotor.setReverseTorqueCurrentLimit(FlywheelConstants.kFlywheelReverseTorqueCurrentLimit);

        // flywheelLeftMotor.setVelocityConversionFactor(FlywheelConstants.kFlywheelGearReduction);
        // flywheelRightMotor.setVelocityConversionFactor(FlywheelConstants.kFlywheelGearReduction);

        flywheelLeftMotor.setInverted(false);
        flywheelRightMotor.setInverted(true);

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

        leftSetpoint = 0;
        rightSetpoint = 0;
        putSmartDashboard();
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

    public double getLeftMotorCurrent(){
        return flywheelLeftMotor.getSupplyCurrent();
    }

    public double getRightMotorCurrent(){
        return flywheelRightMotor.getSupplyCurrent();
    }

    public void stopFlywheel() {
        flywheelLeftMotor.setMotor(0);
        flywheelRightMotor.setMotor(0);
    }

    public void runFlywheelVelocitySetpoint(double leftSpeed, double rightSpeed) {
        runLeftFlywheelVelocitySetpoint(leftSpeed);
        runRightFlywheelVelocitySetpoint(rightSpeed);
    }

    public void runLeftFlywheelVelocitySetpoint(double speed) {
        flywheelLeftMotor.setVelocityTorqueFOC(speed / 60);
        leftSetpoint = speed;
    }

    public void runRightFlywheelVelocitySetpoint(double speed) {
        flywheelRightMotor.setVelocityTorqueFOC(speed / 60);
        rightSetpoint = speed;
    }

    public void runFlywheelAmp() {
        runFlywheelVelocitySetpoint(ScoringConstants.kLeftFlywheelAmpRPM + SmartDashboard.getNumber("AMP Shot Offset RPM", 0), ScoringConstants.kRightFlywheelAmpRPM + SmartDashboard.getNumber("AMP Shot Offset RPM", 0));
    }

    public void runFlywheelHP() {
        runFlywheelVelocitySetpoint(ScoringConstants.kLeftFlywheelHPIntakeRPM,
                ScoringConstants.kRightFlywheelHPIntakeRPM);
    }

    public void runFlywheelLayup() {
        runFlywheelVelocitySetpoint(ScoringConstants.kLeftFlywheelLayupRPM + rpmDelta,
                ScoringConstants.kRightFlywheelLayupRPM + rpmDelta);
    }

    public void runFlywheelLimelight() {
        runFlywheelVelocitySetpoint(ScoringConstants.kLeftFlywheelLLShootingRPM + rpmDelta,
                ScoringConstants.kRightFlywheelLLShootingRPM + rpmDelta);
    }

    public double getFlywheelLeftRPM() {
        return flywheelLeftMotor.getRPM();
    }

    public double getFlywheelRightRPM() {
        return flywheelRightMotor.getRPM();
    }

    public void setRPMDelta(double delta){
        rpmDelta = delta;
    }

    public void putSmartDashboard() {
        // SmartDashboard.putBoolean("Update Flywheel PID", false);
        SmartDashboard.putNumber("AMP Shot Offset RPM", 0);

        // SmartDashboard.putNumber("Flywheel S", FlywheelConstants.kFlywheelS);
        // SmartDashboard.putNumber("Flywheel V", FlywheelConstants.kFlywheelV);
        // SmartDashboard.putNumber("Flywheel A", FlywheelConstants.kFlywheelA);
        // SmartDashboard.putNumber("Flywheel P", FlywheelConstants.kFlywheelP);
        // SmartDashboard.putNumber("Flywheel I", FlywheelConstants.kFlywheelI);
        // SmartDashboard.putNumber("Flywheel D", FlywheelConstants.kFlywheelD);
        // SmartDashboard.putNumber("Flywheel FF", FlywheelConstants.kFlywheelFF);
        // SmartDashboard.putBoolean("Flywheel Percent Output", false);
        // SmartDashboard.putNumber("Flywheel Left Percent Output", 0);
        // SmartDashboard.putNumber("Flywheel Right Percent Output", 0);
    }

    public void updateSmartDashboard() {
        // if (SmartDashboard.getBoolean("Flywheel Percent Output", false)) {
        //     runLeftFlywheelPercentOutput(SmartDashboard.getNumber("Flywheel Left Percent Output", 0));
        //     runRightFlywheelPercentOutput(SmartDashboard.getNumber("Flywheel Right Percent Output", 0));
            // flywheelLeftMotor.setMotor(SmartDashboard.getNumber("Flywheel Left Percent
            // Output", 0));
            // flywheelRightMotor.setMotor(SmartDashboard.getNumber("Flywheel Right Percent
            // Output", 0));
        // }

        // if (SmartDashboard.getBoolean("Update Flywheel PID", false)) {
            // flywheelLeftMotor.setVelocityPIDValues(
            //         SmartDashboard.getNumber("Flywheel S", FlywheelConstants.kFlywheelS),
            //         SmartDashboard.getNumber("Flywheel V", FlywheelConstants.kFlywheelV),
            //         SmartDashboard.getNumber("Flywheel A", FlywheelConstants.kFlywheelA),
            //         SmartDashboard.getNumber("Flywheel P", FlywheelConstants.kFlywheelP),
            //         SmartDashboard.getNumber("Flywheel I", FlywheelConstants.kFlywheelI),
            //         SmartDashboard.getNumber("Flywheel D", FlywheelConstants.kFlywheelD),
            //         SmartDashboard.getNumber("Flywheel FF", FlywheelConstants.kFlywheelFF));

            // flywheelRightMotor.setVelocityPIDValues(
            //         SmartDashboard.getNumber("Flywheel S", FlywheelConstants.kFlywheelS),
            //         SmartDashboard.getNumber("Flywheel V", FlywheelConstants.kFlywheelV),
            //         SmartDashboard.getNumber("Flywheel A", FlywheelConstants.kFlywheelA),
            //         SmartDashboard.getNumber("Flywheel P", FlywheelConstants.kFlywheelP),
            //         SmartDashboard.getNumber("Flywheel I", FlywheelConstants.kFlywheelI),
            //         SmartDashboard.getNumber("Flywheel D", FlywheelConstants.kFlywheelD),
            //         SmartDashboard.getNumber("Flywheel FF", FlywheelConstants.kFlywheelFF));

        //     leftSetpoint = SmartDashboard.getNumber("Flywheel Left Motor RPM Setpoint",
        //     leftSetpoint);
        //     rightSetpoint = SmartDashboard.getNumber("Flywheel Right Motor RPM Setpoint",
        //     rightSetpoint);

        //     flywheelLeftMotor.setVelocityTorqueFOC(leftSetpoint / 60);
        //     flywheelRightMotor.setVelocityTorqueFOC(rightSetpoint / 60);
        // }

        SmartDashboard.putNumber("Flywheel Left RPM", getFlywheelLeftRPM());
        SmartDashboard.putNumber("Flywheel Right RPM", getFlywheelRightRPM());
        SmartDashboard.putBoolean("Flywheel Is At RPM", isAtRPM());
        SmartDashboard.putNumber("Flywheel Left Current", flywheelLeftMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Flywheel Right Current", flywheelRightMotor.getSupplyCurrent());

        SmartDashboard.putNumber("Flywheel Left Motor RPM Setpoint", leftSetpoint);
        SmartDashboard.putNumber("Flywheel Right Motor RPM Setpoint", rightSetpoint);
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }

    private boolean isLeftAtRPM() {
        return (Math.abs(getFlywheelLeftRPM() - leftSetpoint) < ScoringConstants.kFlywheelShotThreshold);
    }

    private boolean isRightAtRPM() {
        return (Math.abs(getFlywheelRightRPM() - rightSetpoint) < ScoringConstants.kFlywheelShotThreshold);
    }

    public boolean isAtRPM() {
        return (isLeftAtRPM() && isRightAtRPM());
    }

}
