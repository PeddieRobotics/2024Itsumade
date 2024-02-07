package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm {

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
        armSecondaryMotor = new Kraken(RobotMap.ARM_SECONDARY_MOTOR, RobotMap.CANIVORE_NAME);

        armPrimaryMotor.setCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);
        armSecondaryMotor.setCurrentLimit(ArmConstants.kArmSecondaryCurrentLimit);

        // this can be overwritten by any other control type
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

    public void periodic() {

    }

}
