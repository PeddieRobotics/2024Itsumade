package frc.robot.subsystems;

import frc.robot.subsystems.LimelightShooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ScoringConstants;
import frc.robot.utils.Constants;
import frc.robot.utils.Conversions;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Rate;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private static Arm instance;
    private LimelightShooter limelightShooter;

    private Kraken armMotor;
    private CANcoder armCANcoder;
    private InterpolatingDoubleTreeMap LLShotMap = new InterpolatingDoubleTreeMap();
    private Rate angle;
    private double gravityFeedForward, armAngleSetpoint, llDistanceMultiplier, ampScoringAngle, lobPassAngle;
    private String stringState;

    public enum ArmState {
        Intaking, Moving, Stowed, Shooting
    }

    private ArmState state, goalState;

    public Arm() {
        limelightShooter = LimelightShooter.getInstance();

        ampScoringAngle = ArmConstants.kArmAmpPosition;
        llDistanceMultiplier = ArmConstants.kArmLLDistMultiplier;
        lobPassAngle = ArmConstants.kArmLobPassPosition;

        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);
        configureCANcoder();

        armMotor = new Kraken(RobotMap.ARM_MOTOR, RobotMap.CANIVORE_NAME);
        armMotor.setInverted(true);
        armMotor.setSupplyCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);
        armMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        armMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);
        armMotor.setBrake();
        armMotor.setEncoder(0);// wont be 0 if measurement is 0 when horizontal

        armMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        armMotor.setRotorToSensorRatio(ArmConstants.kRotorToSensorRatio);
        armMotor.setSensorToMechanismRatio(ArmConstants.kArmSensorToMechanismRatio);

        armMotor.setVelocityPIDValues(ArmConstants.kArmS, ArmConstants.kArmV, ArmConstants.kArmA, ArmConstants.kArmP,
                ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF, ArmConstants.kArmG,
                GravityTypeValue.Arm_Cosine);
        armMotor.setMotionMagicParameters(ArmConstants.kCancoderCruiseVelocityRPS, ArmConstants.kCancoderCruiseMaxAccel,
                ArmConstants.kCancoderCruiseMaxJerk);

        armMotor.setSoftLimits(true, Constants.ArmConstants.kArmForwardSoftLimit,
                Constants.ArmConstants.kArmReverseSoftLimit);

        for (double[] pair : Constants.ScoringConstants.treeMapValues) {
            LLShotMap.put(pair[0], pair[1]);
        }

        armAngleSetpoint = 0;
        state = ArmState.Stowed;
        stringState = "";
        goalState = ArmState.Stowed;
        angle = new Rate(0);
        gravityFeedForward = 0;

        // putSmartDashboard();
    }

    public void configureCANcoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // doublecheck this
        canCoderConfig.MagnetSensor.MagnetOffset = ArmConstants.kArmMagnetOffset; // the value that we put to
                                                                                  // shuffleboard
                                                                                  // is modified from the actual
                                                                                  // CANCoder to multiply by 360 for
                                                                                  // degrees,
                                                                                  // and then divide by 2 for the
                                                                                  // difference between CANCoder and
                                                                                  // actual arm shaft.
                                                                                  // This just undoes that
        armCANcoder.getConfigurator().apply(canCoderConfig);
    }

    public void setArmPercentOutput(double speed) {
        // armMotor.setMotor(speed);
    }

    public void stopArm() {
        // armMotor.setMotor(0);
    }

    public void putSmartDashboard() {
        SmartDashboard.putBoolean("Open Loop Arm Control", false);
        SmartDashboard.putNumber("Arm Percent Output", 0);

        SmartDashboard.putBoolean("Set Arm Setpoint", false);
        SmartDashboard.putNumber("Arm Setpoint", 0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("ARM Absolute CANCoder Reading", getAbsoluteCANCoderPosition());
        SmartDashboard.putNumber("ARM Current Angle (Degrees)", getArmAngleDegrees());
        SmartDashboard.putNumber("ARM Internal Motor Encoder Reading", armMotor.getPosition() * 360);

        SmartDashboard.putNumber("Arm Motor Current", armMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Arm Motor Temperature", armMotor.getMotorTemperature());

        // SmartDashboard.putNumber("Arm Motor Velocity", armMotor.getVelocity());
        // SmartDashboard.putNumber("Motor vel RPS", angle.getVel() * ArmConstants.kRotorToSensorRatio);
        // SmartDashboard.putNumber("Motor Accel RPS^2", angle.getAccel() * ArmConstants.kRotorToSensorRatio);
        // SmartDashboard.putNumber("Motor Jerk RPS^3", angle.getJerk() * ArmConstants.kRotorToSensorRatio);

        // if (SmartDashboard.getBoolean("Open Loop Arm Control", false)) {
        //     armMotor.setMotor(OperatorOI.getInstance().getRightForward());
        // }

        // if (SmartDashboard.getBoolean("Set Arm Setpoint", false)) {
        //     setArmAngle(SmartDashboard.getNumber("Arm Setpoint", 0));
        // }
        // SmartDashboard.putNumber("Converted Arm Setpoint",
        //         Conversions.convertArmDegreesToRotations(SmartDashboard.getNumber("Arm Setpoint", 0)));
    }

    // CANcoder reads 0 to 1, we use this to get easier to read angles to put to
    // dashboard
    // (* 360 for degrees, /2 for difference between CANcoder output shaft and
    // actual arm shaft)
    public double getAbsoluteCANCoderPosition() {
        return armCANcoder.getPosition().getValueAsDouble();
    }

    public double getArmAngleDegrees() {
        return Conversions.convertRotationsToArmDegrees(getAbsoluteCANCoderPosition()/2);
    }

    public double getArmTemperature(){
        return armMotor.getMotorTemperature();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public void RequestState(ArmState requestedState) {
        if (requestedState == state || requestedState == goalState) {
            return;
        }
        state = ArmState.Moving;
        goalState = requestedState;
    }

    public boolean isAtHPAngle() {
        return Math.abs(getArmAngleDegrees() - ArmConstants.kArmIntakeHPPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtStowAngle() {
        return Math.abs(getArmAngleDegrees()
                - ArmConstants.kArmStowPosition) < ArmConstants.kArmStowPositionEpsilon;
    }

    public boolean isAtGroundIntakeAngle() {
        return Math.abs(
                getArmAngleDegrees() - ArmConstants.kArmIntakePositionFromGround) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtLLAngle() {
        return Math.abs(getArmAngleDegrees()-armAngleSetpoint) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtFrontLayupAngle() {
        return Math.abs(
                getArmAngleDegrees() - ArmConstants.kArmFrontLayupPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtAmpScoringAngle(){
        return Math.abs(
                getArmAngleDegrees() - ampScoringAngle) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtSideLayupAngle() {
        return Math.abs(
                getArmAngleDegrees() - ArmConstants.kArmSideLayupPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public void setArmAngle(double angle) {
        armAngleSetpoint = angle;
        armMotor.setMotionMagicTorqueCurrentFOC(Conversions.convertArmDegreesToRotations(angle));
    }

    public double getAngleFromDist(double dist) {
        return LLShotMap.get(dist * llDistanceMultiplier);
    }

    public double getArmAngleSetpoint(){
        return armAngleSetpoint;
    }

    // Methods to Set Arm to a specific position

    public void setGroundIntakePosition() {
        setArmAngle(ArmConstants.kArmIntakePositionFromGround);
    }

    public double getMotorSupplyCurrent(){
        return armMotor.getSupplyCurrent();
    }

    public double getMotorTorqueCurrent(){
        return armMotor.getTorqueCurrent();
    }

    public void setHPIntakePosition() {
        setArmAngle(ArmConstants.kArmIntakeHPPosition);
    }

    public void setAmpPosition() {
        setArmAngle(ampScoringAngle);
    }

    public void setAmpPrepPosition(){
        setArmAngle(ArmConstants.kArmAmpPrepPosition);
    }

    public void setPodiumPosition() {
        setArmAngle((ArmConstants.kArmPodiumShotPosition) * ArmConstants.kArmLLDistMultiplier);
    }

    public void setFrontLayupPosition() {
        setArmAngle(ArmConstants.kArmFrontLayupPosition);
    }

    public void setSideLayupPosition() {
        setArmAngle(ArmConstants.kArmSideLayupPosition);
    }

    public void setLLPosition() {
        SmartDashboard.putNumber("ARM LL Setpoint", getAngleFromDist(limelightShooter.getFilteredDistance()));
        // setArmAngle(getAngleFromDist(limelightShooter.getFilteredDistance()));
        setArmAngle(getAngleFromDist(limelightShooter.getLastDistance()));
    }

    public void setLobPassPosition() {
        setArmAngle(lobPassAngle);
    }

    public void setStowPosition() {
        setArmAngle(ArmConstants.kArmStowPosition);
    }

    public void setLobPassAngle(double angle){
        lobPassAngle = angle;
    }

    public void setArmNeutralMode(){
        armMotor.setNeutralControl();
    }

    public double getLLDistanceMultiplier(){
        return llDistanceMultiplier;
    }

    public void setLLDistanceMultiplier(double multiplier){
        llDistanceMultiplier = multiplier;
    }

    public double getAmpScoringAngle(){
        return ampScoringAngle;
    }

    public void setAmpScoringAngle(double angle){
        ampScoringAngle = angle;
    }

    @Override
    public void periodic() {
        angle.update(getAbsoluteCANCoderPosition());
        updateSmartDashboard();
    }

}
