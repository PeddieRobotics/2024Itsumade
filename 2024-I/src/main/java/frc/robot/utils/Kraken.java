package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kraken {
    private final TalonFX talon;
    private TalonFXConfiguration config;
    private int deviceID;
    private String canbusName;
    private Orchestra orchestra;

    // Open Loop Control
    private double feedForward = 0.0;
    private double velocityConversionFactor = 1.0;

    public Kraken(int deviceID, String canbusName) {
        this.talon = new TalonFX(deviceID, canbusName);
        this.deviceID = deviceID;
        this.canbusName = canbusName;
        this.config = new TalonFXConfiguration();
        talon.getConfigurator().setPosition(0);

        orchestra = new Orchestra();
        orchestra.addInstrument(talon);

        var status = orchestra.loadMusic("output.chrp");
        // factoryReset();
    }

    public void factoryReset() {
        talon.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void setPosition(double position){
        talon.getConfigurator().setPosition(position);
    }

    public void setBrake() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talon.getConfigurator().apply(config);

    }

    public void setCoast() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talon.getConfigurator().apply(config);
    }

    public double getPosition() {
        // Get position from TalonFX and apply position conversion factor
        return talon.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        // Get velocity from TalonFX and apply velocity conversion factor
        return talon.get();
    }

    public double getMPS() {
        return talon.getRotorVelocity().getValueAsDouble() * velocityConversionFactor;
    }

    public void setCurrentLimit(double currentLimit) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        // config.CurrentLimits.StatorCurrentLimitEnable = true;
        // config.CurrentLimits.StatorCurrentLimit = currentLimit;
        
        talon.getConfigurator().apply(config);
    }

    public void setClosedLoopRampRate(double rampRate) {
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        talon.getConfigurator().apply(config);
    }

    public void setMagicMotionParameters(double cruiseVelocity, double maxAcceleration, double maxJerk) {
        config.MotionMagic.MotionMagicJerk = maxJerk;
        config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;

        talon.getConfigurator().apply(config);
    }

    public void setSoftLimits(boolean enableSoftLimit, double forwardLimitValue, double reverseLimitValue) {
        
        if (enableSoftLimit) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitValue;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimitValue;
        } else {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }
        
        talon.getConfigurator().apply(config);
    }

    public void setContinuousOutput(){
        config.ClosedLoopGeneral.ContinuousWrap = true;
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

    public void setPositionConversionFactor(double conversionFactor){
        config.Feedback.SensorToMechanismRatio = conversionFactor;
        talon.getConfigurator().apply(config);
    }

    public void setVelocityConversionFactor(double conversionFactor){
        velocityConversionFactor = conversionFactor;
    }

    public void setRotorToSensorRatio(double conversionRatio){
        config.Feedback.RotorToSensorRatio = conversionRatio;
        talon.getConfigurator().apply(config);
    }

    public void setControlPosition(double position, int slot) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(slot);
        talon.setControl(request.withPosition(position).withEnableFOC(true));
    }

    public void setControlVelocity(double velocity, int slot) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(slot);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withEnableFOC(true));
    }

    public void setPositionWithFeedForward(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward).withEnableFOC(true));
    }

    public void setVelocityWithFeedForward(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withFeedForward(feedForward).withEnableFOC(true));
    }

    public void setFeedbackDevice(int deviceID, FeedbackSensorSourceValue feedbackType){
        config.Feedback.FeedbackRemoteSensorID = deviceID;
        config.Feedback.FeedbackSensorSource = feedbackType;
        talon.getConfigurator().apply(config);
    }

    public double getSupplyCurrent() {
        return talon.getSupplyCurrent().getValueAsDouble();
    }

    public double getMotorTemperature() {
        return talon.getDeviceTemp().getValueAsDouble();
    }

    public void updateSmartdashBoard(){
        SmartDashboard.putNumber(canbusName + " " + deviceID + " motor", 1);
    }

    public void playMusic(boolean on){
        if(on){
            orchestra.play();
        } else {
            orchestra.pause();
        }
    }

}