package frc.robot.utils;

import java.lang.invoke.VolatileCallSite;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Kraken {
    private final TalonFX talon;
    private TalonFXConfiguration config;
    private int deviceID;
    private String canbusName;

    private double positionConversionFactor = 1.0; // Conversion factor for position
    private double velocityConversionFactor = 1.0; // Conversion factor for velocity

    // Open Loop Control
    private double feedForward = 0.0;

    public Kraken(int deviceID, String canbusName) {
        this.talon = new TalonFX(deviceID, canbusName);
        this.deviceID = deviceID;
        this.canbusName = canbusName;
        this.config = new TalonFXConfiguration();
        talon.getConfigurator().setPosition(0);
        // factoryReset();
    }

    public void factoryReset() {
        talon.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void setPosition(double position){
        talon.getConfigurator().setPosition(position);
    }

    public void setFeedbackDevice(){
        config.Feedback.FeedbackRemoteSensorID = 1;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talon.getConfigurator().apply(config);
    }

    public void setBrake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = motorOutputConfigs;

        talon.getConfigurator().apply(config);
    }

    public void setCoast() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = motorOutputConfigs;
        // p

        talon.getConfigurator().apply(config);
    }

    public void setPositionConversionFactor(double conversionFactor) {
        this.positionConversionFactor = conversionFactor;
        // Apply conversion factor for position
    }

    public void setVelocityConversionFactor(double conversionFactor) {
        this.velocityConversionFactor = conversionFactor;
        // Apply conversion factor for velocity
    }

    public double getPosition() {
        // Get position from TalonFX and apply position conversion factor
        return talon.getPosition().getValueAsDouble() * positionConversionFactor;
    }

    public double getVelocity() {
        // Get velocity from TalonFX and apply velocity conversion factor
        return talon.get() * velocityConversionFactor;
    }

    public double getRPM() {
        return talon.getRotorVelocity().getValueAsDouble() * 60;
    }

    public void setCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        // currentLimitsConfig.StatorCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = currentLimit;
        // currentLimitsConfig.StatorCurrentLimit = currentLimit;
        config.CurrentLimits = currentLimitsConfig;

        talon.getConfigurator().apply(config);

    }

    public void setClosedLoopRampRate(double rampRate) {
        ClosedLoopRampsConfigs closedLoopRampRateConfig = new ClosedLoopRampsConfigs();
        closedLoopRampRateConfig.DutyCycleClosedLoopRampPeriod = rampRate;
        closedLoopRampRateConfig.TorqueClosedLoopRampPeriod = rampRate;
        closedLoopRampRateConfig.VoltageClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps = closedLoopRampRateConfig;

        talon.getConfigurator().apply(config);

    }

    public void setMagicMotionParameters(double cruiseVelocity, double maxAcceleration, double maxJerk) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;
        motionMagicConfigs.MotionMagicJerk = maxJerk;
        config.MotionMagic = motionMagicConfigs;

        talon.getConfigurator().apply(config);
    }

    public void setSoftLimits(boolean enableSoftLimit, double forwardLimitValue, double reverseLimitValue) {
        SoftwareLimitSwitchConfigs softwareLimitConfigs = new SoftwareLimitSwitchConfigs();

        if (enableSoftLimit) {
            softwareLimitConfigs.ForwardSoftLimitEnable = true;
            softwareLimitConfigs.ReverseSoftLimitEnable = true;
            softwareLimitConfigs.ForwardSoftLimitThreshold = forwardLimitValue;
            softwareLimitConfigs.ReverseSoftLimitThreshold = reverseLimitValue;
        } else {
            softwareLimitConfigs.ForwardSoftLimitEnable = false;
            softwareLimitConfigs.ReverseSoftLimitEnable = false;
        }

        config.SoftwareLimitSwitch = softwareLimitConfigs;

        talon.getConfigurator().apply(config);
    }

    public void resetEncoder() {
        talon.getConfigurator().setPosition(0);
    }

    public void setMotor(double percentOutput) {
        final DutyCycleOut request = new DutyCycleOut(0);
        // Ensure the percentOutput is within the acceptable range [-1.0, 1.0]
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));

        // Set the control request to the motor controller
        talon.setControl(request.withOutput(percentOutput));

    }

    public void setPIDValues(double kP, double kI, double kD, double kF) {
        var pidSlotConfigs = new Slot0Configs();

        feedForward = kF;
        pidSlotConfigs.kP = kP;
        pidSlotConfigs.kI = kI;
        pidSlotConfigs.kD = kD;
        talon.getConfigurator().apply(pidSlotConfigs);
    }

    public void setVelocityPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF) {
        var pidSlotConfigs = new Slot0Configs();

        feedForward = kF;
        pidSlotConfigs.kS = kS;
        pidSlotConfigs.kV = kV;
        pidSlotConfigs.kA = kA;
        pidSlotConfigs.kP = kP;
        pidSlotConfigs.kI = kI;
        pidSlotConfigs.kD = kD;
        talon.getConfigurator().apply(pidSlotConfigs);
    }

    public void setClosedLoopContinuousInput(){
        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true;
        config.ClosedLoopGeneral = closedLoopGeneralConfigs;
        
        talon.getConfigurator().apply(config);
    }

    public void setControlPosition(double position, int slot) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(slot);
        talon.setControl(request.withPosition(position * positionConversionFactor));
    }

    public void setControlVelocity(double velocity, int slot) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(slot);
        talon.setControl(request.withVelocity(velocity * velocityConversionFactor));
    }

    public void setPositionWithFeedForward(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position * positionConversionFactor).withFeedForward(feedForward));
    }

    public void setVelocityWithFeedForward(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity * velocityConversionFactor).withFeedForward(feedForward));
    }

    public double getSupplyCurrent() {
        return talon.getSupplyCurrent().getValueAsDouble();
    }

    public double getMotorTemperature() {
        return talon.getDeviceTemp().getValueAsDouble();
    }

}