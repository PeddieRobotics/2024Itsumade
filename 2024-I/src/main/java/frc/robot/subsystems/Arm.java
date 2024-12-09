package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;

public class Arm extends SubsystemBase{
    private static Arm arm;
    private Kraken motor;
    private CANcoder cancoder;

    public Arm(){
        motor = new Kraken(Constants.ArmConstants.armMotorID, Constants.GlobalConstants.canivore_name);
        cancoder = new CANcoder(Constants.ArmConstants.armCancoderID, Constants.GlobalConstants.canivore_name);
        configureCANCoder();

        motor.setInverted(true);
        motor.setSupplyCurrentLimit(Constants.ArmConstants.armCurrentLimit);
        motor.setForwardTorqueCurrentLimit(Constants.ArmConstants.armTorqueLimit);
        motor.setReverseTorqueCurrentLimit(-Constants.ArmConstants.armTorqueLimit);
        motor.setBrake();
        motor.setEncoder(0);

        motor.setFeedbackDevice(Constants.ArmConstants.armCancoderID, FeedbackSensorSourceValue.FusedCANcoder);
        motor.setRotorToSensorRatio(Constants.ArmConstants.armRotorToSensorRatio);
        motor.setSensorToMechanismRatio(Constants.ArmConstants.armSensorToMechanismRatio);
        motor.setVelocityPIDValues(0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);

        motor.setSoftLimits(true, Constants.ArmConstants.armForwardSoftLimit, Constants.ArmConstants.armReverseSoftLimit);
    }

    public void configureCANCoder(){
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = Constants.ArmConstants.armMagnetOffset;

        cancoder.getConfigurator().apply(canCoderConfig);
    }

    public static Arm getInstance(){
       if(arm == null){
            arm = new Arm();
       }
      return arm;
    }
}
