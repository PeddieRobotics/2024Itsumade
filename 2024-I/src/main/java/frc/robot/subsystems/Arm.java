package frc.robot.subsystems;

import frc.robot.subsystems.LimelightFront;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm {

    private static Arm instance;
    private static LimelightFront limelightFront;

    private Kraken armPrimaryMotor, armSecondaryMotor;
    private CANcoder armCANcoder;
    private InterpolatingDoubleTreeMap LLShotMap = new InterpolatingDoubleTreeMap();

    public enum ArmState {
        Intaking, Moving, Stowed, Shooting
    }

    private ArmState state, goalState;

    public Arm() {
        limelightFront = LimelightFront.getInstance();

        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);

        armPrimaryMotor = new Kraken(RobotMap.ARM_PRIMARY_MOTOR, RobotMap.CANIVORE_NAME);
        armSecondaryMotor = new Kraken(RobotMap.ARM_SECONDARY_MOTOR, RobotMap.CANIVORE_NAME);

        armPrimaryMotor.setCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);
        armSecondaryMotor.setCurrentLimit(ArmConstants.kArmSecondaryCurrentLimit);

        // this can be overwritten by any other control type, follower
        armSecondaryMotor.setFollower(RobotMap.ARM_PRIMARY_MOTOR, false);

        armPrimaryMotor.setCoast();
        armSecondaryMotor.setCoast();

        armPrimaryMotor.setSoftLimits(true, ArmConstants.kArmForwardSoftLimit, ArmConstants.kArmReverseSoftLimit);

        armPrimaryMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.RemoteCANcoder);
        armSecondaryMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.RemoteCANcoder);

        armPrimaryMotor.setPositionConversionFactor(ArmConstants.kArmPositionConversionFactor);
        armSecondaryMotor.setPositionConversionFactor(ArmConstants.kArmPositionConversionFactor);

        armPrimaryMotor.setPIDValues(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF);
        armSecondaryMotor.setPIDValues(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF);

        for(double[] pair:Constants.ScoringConstants.treeMapValues){
            LLShotMap.put(pair[0],pair[1]);
        }

        state = ArmState.Stowed;
        goalState = ArmState.Stowed;
    }

    public void configureCANcoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // doublecheck this
        armCANcoder.getConfigurator().apply(canCoderConfig);
    }

    public void setArmPercentOutput(double speed) {
        armPrimaryMotor.setMotor(speed);
    }

    public void stopArm() {
        armPrimaryMotor.setMotor(0);
    }

    public void putSmartDashboard() {
        SmartDashboard.putBoolean("Update Arm PID", false);
        SmartDashboard.putNumber("Arm Primary Motor Position Setpoint", 0);

        SmartDashboard.putNumber("Arm P", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm I", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm D", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm FF", ArmConstants.kArmFF);
    }

    public void updateSmartDashboard() {
        if (SmartDashboard.getBoolean("Update Arm PID", false)) {
            armPrimaryMotor.setPIDValues(
                    SmartDashboard.getNumber("Arm P", ArmConstants.kArmP),
                    SmartDashboard.getNumber("Arm I", ArmConstants.kArmI),
                    SmartDashboard.getNumber("Arm D", ArmConstants.kArmD),
                    SmartDashboard.getNumber("Arm FF", ArmConstants.kArmFF));

            armSecondaryMotor.setPIDValues(
                    SmartDashboard.getNumber("Arm P", ArmConstants.kArmP),
                    SmartDashboard.getNumber("Arm I", ArmConstants.kArmI),
                    SmartDashboard.getNumber("Arm D", ArmConstants.kArmD),
                    SmartDashboard.getNumber("Arm FF", ArmConstants.kArmFF));

           armPrimaryMotor.setPositionWithFeedForward(SmartDashboard.getNumber("Arm Primary Motor Position Setpoint", 0));
        }
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

    public boolean isAtHPAngle(){
        return Math.abs(getArmAngleDegrees() - ArmConstants.kArmIntakeHPPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtGroundIntakeAngle(){
        return Math.abs(getArmAngleDegrees() - ArmConstants.kArmIntakePositionFromGround) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtLLAngle(){
        return Math.abs(getArmAngleDegrees() - getAngleFromDist(limelightFront.getDistance())) < ArmConstants.kArmPositionEpsilon;
    }

     public void setArmAngle(double angle){
        armPrimaryMotor.setPosition(angle);
    }

    public double getArmAngleDegrees(){
        return armCANcoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public double getAngleFromDist(double dist){
        return LLShotMap.get(dist);
    }

    //Methods to Set Arm to a specific position

    public void setGroundIntakePosition(){
        setArmAngle(ArmConstants.kArmIntakePositionFromGround);
    }

    public void setHPIntakePosition(){
        setArmAngle(ArmConstants.kArmIntakeHPPosition);
    }

    public void setAmpPosition(){
        setArmAngle(ArmConstants.kArmAmpPosition);
    }

    public void setLayupPosition() {
        setArmAngle(ArmConstants.kArmLayupPosition);
    }

    public void setLLPosition(){
        setArmAngle(getAngleFromDist(limelightFront.getDistance())); 
    }

    public void setStowPosition(){
        setArmAngle(ArmConstants.kArmStowPosition);
    }

    // public void getArmAngle(){
    //     armPrimaryMotor.getPosition();
    // }

    public void periodic() {

    }
}
