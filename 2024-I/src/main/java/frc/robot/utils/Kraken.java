package frc.robot.utils;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Kraken {
    private TalonFX talon;
    private TalonFXConfigurator configurator;
    private TalonFXConfiguration config;
    private int deviceID;
    private String canbusName;

    private double positionConversionFactor = 1.0; // Conversion factor for position
    private double velocityConversionFactor = 1.0; // Conversion factor for velocity

    // Open Loop Control
    private DutyCycleOut motorRequest = new DutyCycleOut(0.0);
    private double feedForward = 0.0;

    public Kraken(int deviceID, String canbusName) {
        this.deviceID = deviceID;
        this.canbusName = canbusName;
        this.talon = new TalonFX(deviceID, canbusName);
        this.config = new TalonFXConfiguration();
    }

    public void setBrake() {
        talon.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoast() {
        talon.setNeutralMode(NeutralModeValue.Coast);
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
        return talon.get() * positionConversionFactor;
    }

    public double getVelocity() {
        // Get velocity from TalonFX and apply velocity conversion factor
        return talon.get() * velocityConversionFactor;
    }

    public void setCurrentLimit(double currentLimit) {
        CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits = currentLimitsConfig;

        configurator.apply(config);
    }

    public void setMotor(double percentOutput) {
        // Ensure the percentOutput is within the acceptable range [-1.0, 1.0]
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));

        // Update the DutyCycleOut control request with the percent output
        motorRequest.withOutput(percentOutput);

        // Set the control request to the motor controller
        talon.setControl(motorRequest);
    }

    public void setPIDValues(double kP, double kI, double kD, double kF) {
        feedForward = kF;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        configurator.apply(config);
    }

    public void setControlPosition(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position));
    }

    public void setControlVelocity(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity));
    }

    public void setPositionWithFeedForward(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward));
    }
    

    public void setVelocityWithFeedForward(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity).withFeedForward(feedForward));
    }
    
}
