package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.utils.Constants.*;

public final class CTREConfig {
    private final TalonFXConfiguration drivingConfig;
    private final TalonFXConfiguration steeringConfig;
    private final CANcoderConfiguration cancoderConfig;

    public CTREConfig(){
        drivingConfig = new TalonFXConfiguration();
        steeringConfig = new TalonFXConfiguration();
        cancoderConfig = new CANcoderConfiguration();

        // Driving Current Limits
        CurrentLimitsConfigs drivingLimitConfig = new CurrentLimitsConfigs();
        drivingLimitConfig.SupplyCurrentLimitEnable = true; // This should always be true
        drivingLimitConfig.SupplyCurrentLimit = DriveConstants.kDriveCurrentLimit;
        drivingConfig.CurrentLimits = drivingLimitConfig;

        // Steer Current Limits
        CurrentLimitsConfigs steerLimitConfig = new CurrentLimitsConfigs();
        steerLimitConfig.SupplyCurrentLimitEnable = true; // This should always be true
        steerLimitConfig.SupplyCurrentLimit = DriveConstants.kTurningCurrentLimit;
        steeringConfig.CurrentLimits = steerLimitConfig;

    }
}
