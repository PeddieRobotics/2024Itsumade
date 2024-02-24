package frc.robot.subsystems;

import frc.robot.subsystems.LimelightShooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Rate;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private static Arm instance;
    private static LimelightShooter limelightShooter;

    private Kraken armMotor;
    private CANcoder armCANcoder;
    private InterpolatingDoubleTreeMap LLShotMap = new InterpolatingDoubleTreeMap();
    private Rate angle;
    private double gravityFeedForward;

    public enum ArmState {
        Intaking, Moving, Stowed, Shooting
    }

    private ArmState state, goalState;

    public Arm() {
        limelightShooter = LimelightShooter.getInstance();

        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);
        configureCANcoder();

        armMotor = new Kraken(RobotMap.ARM_MOTOR, RobotMap.CANIVORE_NAME);

        armMotor.setInverted(true);
        armMotor.setCurrentLimit(ArmConstants.kArmPrimaryCurrentLimit);
        armMotor.setBrake();
        armMotor.setEncoder(0);//wont be 0 if measurement is 0 when horizonal

        armMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        armMotor.setRotorToSensorRatio(ArmConstants.kRotorToSensorRatio);
        armMotor.setSensorToMechanismRatio(ArmConstants.kArmSensorToMechanismRatio);
        // armMotor.setPositionConversionFactor(1.0);

        armMotor.setVelocityPIDValues(ArmConstants.kArmS, ArmConstants.kArmV, ArmConstants.kArmA, ArmConstants.kArmP,
                ArmConstants.kArmI, ArmConstants.kArmD, ArmConstants.kArmFF); // recaculate offset so 0 is horizontal
        armMotor.setMotionMagicParameters(ArmConstants.kCancoderCruiseVelocityRPS, ArmConstants.kCancoderCruiseMaxAccel,
                ArmConstants.kCancoderCruiseMaxJerk);

        armMotor.setSoftLimits(true,
                (Constants.ArmConstants.kArmForwardSoftLimitDegrees) / 360,
                (Constants.ArmConstants.kArmReverseSoftLimitDegrees) / 360);

        for (double[] pair : Constants.ScoringConstants.treeMapValues) {
            LLShotMap.put(pair[0], pair[1]);
        }

        state = ArmState.Stowed;
        goalState = ArmState.Stowed;
        angle = new Rate(0);
        gravityFeedForward = 0;

        putSmartDashboard();
    }

    // TODO: update these constants
    public void configureCANcoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // doublecheck this
        canCoderConfig.MagnetSensor.MagnetOffset = 2 * Constants.ArmConstants.kArmPositionOffsetDegrees / 360; // the value that we put to shuffleboard
                                                                                                                // is modified from the actual CANCoder to multiply by 360 for degrees, 
                                                                                                                //and then divide by 2 for the difference between CANCoder and actual arm shaft.
                                                                                                                //This just undoes that
        armCANcoder.getConfigurator().apply(canCoderConfig);
    }

    public void setArmPercentOutput(double speed) {
        armMotor.setMotor(speed);
    }

    public void stopArm() {
        armMotor.setMotor(0);
    }

    public void putSmartDashboard() {
        SmartDashboard.putBoolean("Open Loop Arm Control", false);
        SmartDashboard.putBoolean("Update Arm PID", false);
        SmartDashboard.putNumber("Arm Position Setpoint Degrees", 0);

        SmartDashboard.putNumber("Arm kG", ArmConstants.kArmG);
        SmartDashboard.putNumber("Arm kS", ArmConstants.kArmS);
        SmartDashboard.putNumber("Arm kV", ArmConstants.kArmV);
        SmartDashboard.putNumber("Arm kA", ArmConstants.kArmA);

        SmartDashboard.putNumber("Arm P", ArmConstants.kArmP);
        SmartDashboard.putNumber("Arm I", ArmConstants.kArmI);
        SmartDashboard.putNumber("Arm D", ArmConstants.kArmD);
        SmartDashboard.putNumber("Arm FF", ArmConstants.kArmFF);

        SmartDashboard.putNumber("Arm Max Cruise Velocity", ArmConstants.kCancoderCruiseVelocityRPS);
        SmartDashboard.putNumber("Arm Max Cruise Accel", ArmConstants.kCancoderCruiseMaxAccel);
        SmartDashboard.putNumber("Arm Max Cruise Jerk", ArmConstants.kCancoderCruiseMaxJerk);

        SmartDashboard.putNumber("Arm Percent Output", 0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("ARM Absolute CANCoder Reading", getAbsoluteCANCoderPosition());
        // SmartDashboard.putNumber("ARM Internal Motor Encoder Reading", armMotor.getPosition() * 360);
        SmartDashboard.putNumber("ARM Internal Motor Encoder Reading", armMotor.getPosition() * 360);

        SmartDashboard.putNumber("Arm Motor Current", armMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Arm Motor Temperature", armMotor.getMotorTemperature());

        SmartDashboard.putNumber("Arm Motor Velocity", armMotor.getVelocity());
        SmartDashboard.putNumber("Motor vel RPS", angle.getVel() * ArmConstants.kRotorToSensorRatio);
        SmartDashboard.putNumber("Motor Accel RPS^2", angle.getAccel() * ArmConstants.kRotorToSensorRatio);
        SmartDashboard.putNumber("Motor Jerk RPS^3", angle.getJerk() * ArmConstants.kRotorToSensorRatio);
        SmartDashboard.putNumber("Arm Calculated FF", getFeedForward(SmartDashboard.getNumber("Arm kG", 0)));

        SmartDashboard.putNumber("Open Loop Speed Input", OperatorOI.getInstance().getRightForward() / 2);
        if (SmartDashboard.getBoolean("Open Loop Arm Control", false)) {
            setArmPercentOutput(OperatorOI.getInstance().getRightForward() / 2);
        }

        SmartDashboard.putNumber("Calculated Arm FF", getFeedForward(SmartDashboard.getNumber("Arm kG", 0)));

        if (SmartDashboard.getBoolean("Update Arm PID", false)) {
            armMotor.setMotionMagicParameters(
                    SmartDashboard.getNumber("Arm Max Cruise Velocity", ArmConstants.kCancoderCruiseVelocityRPS),
                    SmartDashboard.getNumber("Arm Max Cruise Accel", ArmConstants.kCancoderCruiseMaxAccel),
                    SmartDashboard.getNumber("Arm Max Cruise Jerk", ArmConstants.kCancoderCruiseMaxJerk));

            armMotor.setVelocityPIDValues(
                    SmartDashboard.getNumber("Arm kS", 0),
                    SmartDashboard.getNumber("Arm kV", 0),
                    SmartDashboard.getNumber("Arm kA", 0),
                    SmartDashboard.getNumber("Arm P", 0),
                    SmartDashboard.getNumber("Arm I", 0),
                    SmartDashboard.getNumber("Arm D", 0),
                    getFeedForward(SmartDashboard.getNumber("Arm kG", 0)));
 
            //armMotor.setPositionTorqueFOC(SmartDashboard.getNumber("Arm Position Setpoint", 0)/360);
            armMotor.setPositionWithFeedForward(SmartDashboard.getNumber("Arm Position Setpoint Degrees", 0)/360);
        }
    }

    public void armPrimarySetPositionMotionMagic(double position) {
        armMotor.setPositionMotionMagic(position);
    }

    public void armPrimarySetVelocityPIDValues(double kS, double kV, double kA, double kP, double kI, double kD,
            double kFF) {
        armMotor.setVelocityPIDValues(kS, kV, kA, kP, kI, kD, kFF);
    }

    public double getFeedForward(double kG){
        return kG * Math.sin(((getAbsoluteCANCoderPosition()+18.75)/180) * Math.PI);
    }
    
    //CANcoder reads 0 to 1, we use this to get easier to read angles to put to dashboard 
    //(* 360 for degrees, /2 for difference between CANcoder output shaft and actual arm shaft)
    public double getAbsoluteCANCoderPosition() {
        return armCANcoder.getPosition().getValueAsDouble() * 360 / 2;
        //return armCANcoder.getPosition().getValueAsDouble();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        // return instance;
        return null;
    }

    public void RequestState(ArmState requestedState) {
        if (requestedState == state || requestedState == goalState) {
            return;
        }
        state = ArmState.Moving;
        goalState = requestedState;
    }

    public boolean isAtHPAngle() {
        return Math.abs(getArmAngleDegrees() - ArmConstants.kArmIntakeHPPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtStowAngle() {
        return Math.abs(armCANcoder.getAbsolutePosition().getValueAsDouble() * 360
                - ArmConstants.kArmStowPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtGroundIntakeAngle() {
        return Math.abs(
                getArmAngleDegrees() - ArmConstants.kArmIntakePositionFromGround) < ArmConstants.kArmPositionEpsilon;
    }

    public boolean isAtLLAngle() {
        return Math.abs(getArmAngleDegrees()
                - getAngleFromDist(limelightShooter.getDistance())) < ArmConstants.kArmPositionEpsilon;
    }

    public void setArmAngle(double angle) {
        armMotor.setPositionWithFeedForward(angle);
    }

    public double getArmAngleDegrees() {
        return armCANcoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public double getAngleFromDist(double dist) {
        return LLShotMap.get(dist);
    }

    // Methods to Set Arm to a specific position

    public void setGroundIntakePosition() {
        setArmAngle(ArmConstants.kArmIntakePositionFromGround);
    }

    public void setHPIntakePosition() {
        setArmAngle(ArmConstants.kArmIntakeHPPosition);
    }

    public void setAmpPosition() {
        setArmAngle(ArmConstants.kArmAmpPosition);
    }

    public void setLayupPosition() {
        setArmAngle(ArmConstants.kArmLayupPosition);
    }

    public void setLLPosition() {
        setArmAngle(getAngleFromDist(limelightShooter.getDistance()));
    }

    public void setStowPosition() {
        setArmAngle(ArmConstants.kArmStowPosition);
    }

    @Override
    public void periodic() {
        angle.update(getAbsoluteCANCoderPosition());
        updateSmartDashboard();
    }
}
