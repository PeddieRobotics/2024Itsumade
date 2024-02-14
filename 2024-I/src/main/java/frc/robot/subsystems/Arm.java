package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    private static Arm instance;

    private Kraken armPrimaryMotor, armSecondaryMotor;
    private CANcoder armCANcoder;

    public enum ArmState {
        Intaking, Moving, Stowed, Shooting
    }

    private ArmState state, goalState;

    public Arm() {
        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);

        armPrimaryMotor = new Kraken(RobotMap.ARM_PRIMARY_MOTOR, RobotMap.CANIVORE_NAME);

        armPrimaryMotor.setInverted(true);
        // armSecondaryMotor = new Kraken(RobotMap.ARM_SECONDARY_MOTOR, RobotMap.CANIVORE_NAME);

        armPrimaryMotor.setCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);
        // armSecondaryMotor.setCurrentLimit(ArmConstants.kArmSecondaryCurrentLimit);

        // this can be overwritten by any other control type
        // armSecondaryMotor.setFollower(RobotMap.ARM_PRIMARY_MOTOR, false);

        armPrimaryMotor.setCoast();

        armPrimaryMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        armPrimaryMotor.setRotorToSensorRatio(Constants.ArmConstants.kRotorToSensorRatio);

        armPrimaryMotor.setPositionConversionFactor(ArmConstants.kArmPositionConversionFactor);

        armPrimaryMotor.setPIDValues(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF);


        state = ArmState.Stowed;
        goalState = ArmState.Stowed;

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

        SmartDashboard.putNumber("Arm P", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm I", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm D", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm FF", ArmConstants.kArmFF);

        SmartDashboard.putNumber("Arm Percent Output", 0);
    }

    public void updateSmartDashboard() {
        if (SmartDashboard.getBoolean("Update Arm PID", false)) {
            armPrimaryMotor.setPIDValues(
                    SmartDashboard.getNumber("Arm P", ArmConstants.kArmP),
                    SmartDashboard.getNumber("Arm I", ArmConstants.kArmI),
                    SmartDashboard.getNumber("Arm D", ArmConstants.kArmD),
                    SmartDashboard.getNumber("Arm FF", ArmConstants.kArmFF));

            // armSecondaryMotor.setPIDValues(
            //         SmartDashboard.getNumber("Arm P", ArmConstants.kArmP),
            //         SmartDashboard.getNumber("Arm I", ArmConstants.kArmI),
            //         SmartDashboard.getNumber("Arm D", ArmConstants.kArmD),
            //         SmartDashboard.getNumber("Arm FF", ArmConstants.kArmFF));

           armPrimaryMotor.setPositionWithFeedForward(SmartDashboard.getNumber("Arm Primary Motor Position Setpoint", 0) / 360.0);
        } else {
            setArmPercentOutput(SmartDashboard.getNumber("Arm Percent Output", 0));
        }

        SmartDashboard.putNumber("cancoder reading", armCANcoder.getAbsolutePosition().getValueAsDouble()*360);
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
        updateSmartDashboard();
    }

    public void layup() {

    }

    public void llalign() {

    }

}
