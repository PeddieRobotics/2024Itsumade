package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.FlywheelConstants;

public class Flywheel {

    private static Flywheel instance;
    private double flywheelSetpoint = 0;

    public Kraken flywheelLeftMotor, flywheelRightMotor;

    public Flywheel() {
        flywheelLeftMotor = new Kraken(RobotMap.FLYWHEEL_LEFT_MOTOR, RobotMap.CANIVORE_NAME);
        flywheelRightMotor = new Kraken(RobotMap.FLYWHEEL_RIGHT_MOTOR, RobotMap.CANIVORE_NAME);

        flywheelLeftMotor.setCurrentLimit(FlywheelConstants.kFlywheelLeftCurrentLimit);
        flywheelRightMotor.setCurrentLimit(FlywheelConstants.kFlywheelRightCurrentLimit);

        flywheelLeftMotor.setVelocityPIDValues(
                FlywheelConstants.kFlywheelS,
                FlywheelConstants.kFlywheelV,
                FlywheelConstants.kFlywheelA,
                FlywheelConstants.kFlywheelP,
                FlywheelConstants.kFlywheelI,
                FlywheelConstants.kFlywheelD,
                FlywheelConstants.kFlywheelFF);

        flywheelRightMotor.setVelocityPIDValues(
                FlywheelConstants.kFlywheelS,
                FlywheelConstants.kFlywheelV,
                FlywheelConstants.kFlywheelA,
                FlywheelConstants.kFlywheelP,
                FlywheelConstants.kFlywheelI,
                FlywheelConstants.kFlywheelD,
                FlywheelConstants.kFlywheelFF);
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    public void runFlywheelPercentOutput(double speed) {
        flywheelLeftMotor.setMotor(speed);
        flywheelRightMotor.setMotor(speed);
    }

    public void runLeftFlywheelPercentOutput(double speed) {
        flywheelLeftMotor.setMotor(speed);
    }

    public void runRightFlywheelPercentOutput(double speed) {
        flywheelRightMotor.setMotor(speed);
    }

    public void stopFlywheel() {
        flywheelLeftMotor.setMotor(0);
        flywheelRightMotor.setMotor(0);
    }

    public void periodic() {

    }

}
