package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.Rate;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    private static Arm instance;

    private Kraken armPrimaryMotor, armSecondaryMotor;
    private CANcoder armCANcoder;
    private Rate angle;

    public enum ArmState {
        Intaking, Moving, Stowed, Shooting
    }

    private ArmState state, goalState;

    public Arm() {
        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);

        armPrimaryMotor = new Kraken(RobotMap.ARM_PRIMARY_MOTOR, RobotMap.CANIVORE_NAME);

        // armPrimaryMotor.setSoftLimits(true, Constants.ArmConstants.kArmForwardSoftLimitDegrees/360, Constants.ArmConstants.kArmReverseSoftLimitDegrees/360);

        armPrimaryMotor.setInverted(true);

        armPrimaryMotor.setCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);

        armPrimaryMotor.setCoast();

        armPrimaryMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);

        armPrimaryMotor.setRotorToSensorRatio(Constants.ArmConstants.kRotorToSensorRatio);

        armPrimaryMotor.setPositionConversionFactor(ArmConstants.kArmPositionConversionFactor);

        armPrimaryMotor.setVelocityPIDValues(ArmConstants.kArmS,ArmConstants.kArmV,ArmConstants.kArmA,ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF);

        armPrimaryMotor.setMotionMagicParameters(ArmConstants.cancoderCruiseVelocityRPS, ArmConstants.cancoderCruiseMaxAccel, ArmConstants.cancoderCruiseMaxJerk);

        state = ArmState.Stowed;
        goalState = ArmState.Stowed;
        angle=new Rate(0);

        configureCANcoder();
        putSmartDashboard();
    }

    public void configureCANcoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // doublecheck this
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.kArmPositionOffsetDegrees/360;
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

        SmartDashboard.putNumber("Arm kS", ArmConstants.kArmS);
        SmartDashboard.putNumber("Arm P", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm I", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm D", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm FF", ArmConstants.kArmFF);

        SmartDashboard.putNumber("Arm Percent Output", 0);
    }

    public void armPrimarySetPositionMotionMagic(double position){
        armPrimaryMotor.setPositionMotionMagic(position);
    }

    public void armPrimarySetVelocityPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kFF){
        armPrimaryMotor.setVelocityPIDValues(kS, kV, kA, kP, kI, kD, kFF);
    }

    public double getAbsoluteCANCoderPosition(){
        return armCANcoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Motor vel RPS",angle.getVel()*ArmConstants.kRotorToSensorRatio);
        SmartDashboard.putNumber("Motor Accel RPS^2",angle.getAccel()*ArmConstants.kRotorToSensorRatio);
        SmartDashboard.putNumber("Motor Jerk RPS^3",angle.getJerk()*ArmConstants.kRotorToSensorRatio);
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

    public boolean canIntake(){
        return Math.abs(armCANcoder.getAbsolutePosition().getValueAsDouble()*360 - Constants.ArmConstants.kArmIntakePosition) < Constants.ArmConstants.kArmPositionEpsilon;
    }



    public void setIntakePosition(){

    }

    public void setHPIntakePosition(){

    }

    public void setAmpPosition(){

    }

    public void setStowPosition(){
        
    }

    @Override
    public void periodic() {
        angle.update(armCANcoder.getPosition().getValueAsDouble());
        updateSmartDashboard();
    }

    public void layup() {

    }

    public void llalign() {

    }

}
