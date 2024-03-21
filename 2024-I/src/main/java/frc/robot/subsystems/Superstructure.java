package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.OdometryTarget;
import frc.robot.subsystems.Lights.LightState;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.ScoringConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private final Arm arm;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Hopper hopper;
    private final Drivetrain drivetrain;
    private final Lights lights;
    // private final CANdle candle;

    private double stateDuration;
    private double internalStateTimer;
    private double customShotAngle;
    private double latestIntakeTime;
    private boolean isIndexedOverride, hasGamepieceOverride, justIntaked;
    private Timer timer;

    public enum SuperstructureState {
        LL_TEST,
        STOW,
        GROUND_INTAKE,
        OUTTAKE,
        HP_INTAKE,
        AMP_PREP,
        AMP_SCORING,
        FRONT_LAYUP_PREP,
        SIDE_LAYUP_PREP,
        LAYUP_SCORING,
        LL_PREP,
        LL_SCORING,
        CUSTOM_SHOT_PREP,
        POST_SCORING,
        PODIUM_PREP,
        PODIUM_SCORING,
        LOB_PASS_PREP,
        LOB_PASSING
    }

    SuperstructureState systemState;
    SuperstructureState nextSystemState;
    SuperstructureState requestedSystemState;

    public Superstructure() {
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        drivetrain = Drivetrain.getInstance();
        lights = Lights.getInstance();

        timer = new Timer();

        systemState = SuperstructureState.STOW;
        nextSystemState = SuperstructureState.STOW;
        requestedSystemState = SuperstructureState.STOW;
        isIndexedOverride = false;

        // candle = new CANdle(0);

        SmartDashboard.putBoolean("Piece Indexed Override", isIndexedOverride); // overrides, just in case
        // hasGamepieceOverride = SmartDashboard.putBoolean("Has Gamepiece Override",
        // false);

        SmartDashboard.putString("STATE", systemState.toString());
        stateDuration = 0;
        internalStateTimer = 0;
        latestIntakeTime = 0;

        SmartDashboard.putBoolean("LL Shot Move Arm", false);
        SmartDashboard.putNumber("LL Shot Angle", 0);

        lights.requestState(LightState.IDLE);
    }

    public static Superstructure getInstance() {
        if (superstructure == null) {
            superstructure = new Superstructure();
        }
        return superstructure;
    }

    public void requestState(SuperstructureState request) {
        requestedSystemState = request;
    }

    public void requestState(SuperstructureState request, double val) {
        if (request == SuperstructureState.CUSTOM_SHOT_PREP) {
            requestedSystemState = request;
            customShotAngle = val;
        }
        return; // throw error or smth? idk
    }

    public String getRobotState() {
        return systemState.toString();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("STATE", systemState.toString());
        SmartDashboard.putBoolean("INDEXED?", isGamepieceIndexed());

        switch (systemState) {
            case LL_TEST:
                if (SmartDashboard.getBoolean("LL Shot Move Arm", false)) {
                    arm.setArmAngle(SmartDashboard.getNumber("LL Shot Angle", 0));
                }
                flywheel.runFlywheelLimelight();

                if (flywheel.isAtRPM()) {
                    hopper.feedFlywheelSpeaker();
                }

                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                }
                break;

            // idle state of robot, arm is in stow position,
            case STOW:
                SmartDashboard.putBoolean("ARM At Stow Angle", arm.isAtStowAngle());

                if (arm.isAtStowAngle()) {
                    Logger.getInstance().logEvent("arm at neutral mode", true);
                    arm.setArmNeutralMode();
                } else {
                    Logger.getInstance().logEvent("arm at neutral mode", false);
                    arm.setStowPosition();
                }

                if (DriverStation.isAutonomous()) {
                    flywheel.runFlywheelLimelight();
                } else {
                    flywheel.stopFlywheel();

                    if (Math.abs(Timer.getFPGATimestamp() - latestIntakeTime) > 1.0) {
                        LimelightHelper.setLEDMode_PipelineControl("limelight-intake");
                        // candle.clearAnimation(0);
                        justIntaked = false;
                    }
                }
                intake.stopIntake();
                hopper.stopHopper();

                if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }

                if (requestedSystemState == SuperstructureState.LL_TEST) {
                    nextSystemState = requestedSystemState;
                }

                break;

            case GROUND_INTAKE:
                arm.setGroundIntakePosition();
                hopper.runHopperGroundIntake();

                if (arm.isAtGroundIntakeAngle()) {
                    if (intake.getSensor()) {
                        intake.runIntakeFeed();
                    } else {
                        intake.runIntake();
                    }
                }

                if (!DriverStation.isAutonomous()) {
                    flywheel.stopFlywheel();
                } else {
                    flywheel.runFlywheelLimelight();
                }

                if(intake.getSensor()){
                    lights.requestState(LightState.INTOOK);
                } else if (lights.getLightState() != LightState.INTOOK){
                    lights.requestState(LightState.INTAKING);
                }

                if (isGamepieceIndexed()) {
                    intake.stopIntake();
                    hopper.stopHopper();
                    latestIntakeTime = Timer.getFPGATimestamp();
                    justIntaked = true;

                    if (!DriverStation.isAutonomous()) {
                        requestState(SuperstructureState.STOW);

                        LimelightHelper.setLEDMode_ForceBlink("limelight-intake");

                        // candle.animate(new StrobeAnimation(125, 35, 250), 0);
                        // candle.setLEDs(125, 35, 250);
                    }
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }

                break;

            case OUTTAKE:
                arm.setGroundIntakePosition();
                intake.reverseIntake();
                if (arm.isAtGroundIntakeAngle()) {
                    hopper.runHopperOuttake();
                }

                if (!DriverStation.isAutonomous()) {
                    flywheel.stopFlywheel();
                } else {
                    flywheel.runFlywheelLimelight();
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                }

                break;

            case HP_INTAKE:
                if (!hopper.getTopSensor() || !hopper.getBottomSensor()) {
                    arm.setHPIntakePosition();
                    flywheel.runFlywheelHP();
                    intake.stopIntake();
                } else if (hopper.getBottomSensor()) {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    requestState(SuperstructureState.STOW);
                    lights.requestState(LightState.IDLE);
                    break;
                }

                // only switch states from intake if done indexing, but let it go into stow from
                // anywhere
                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case AMP_PREP:
                arm.setAmpPrepPosition();
                flywheel.runFlywheelAmp();
                hopper.stopHopper();
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case AMP_SCORING:
                arm.setAmpPosition();    
                if (!timer.hasElapsed(ScoringConstants.kShootingStateTime) && arm.isAtAmpScoringAngle()) { // stop this once the piece is scored
                    flywheel.runFlywheelAmp();
                    hopper.feedFlywheelAmp();
                    intake.stopIntake();
                    timer.start();
                } else if (!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    Logger.getInstance().logEvent("Amp shot finished, arm angle " + arm.getArmAngleDegrees(), false);
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.reset();
                    requestState(SuperstructureState.POST_SCORING);
                    break;
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.POST_SCORING) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case PODIUM_PREP:
                arm.setPodiumPosition();
                flywheel.runFlywheelLimelight();
                hopper.stopHopper();
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case PODIUM_SCORING:
                if (!timer.hasElapsed(ScoringConstants.kShootingStateTime)) { // stop this once the piece is scored
                    flywheel.runFlywheelLimelight();
                    hopper.feedFlywheelPodium();
                    intake.stopIntake();
                    timer.start();
                } else if (!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    Logger.getInstance().logEvent("Podium shot finished, arm angle " + arm.getArmAngleDegrees(), false);
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.reset();
                    lights.requestState(LightState.IDLE);
                    requestState(SuperstructureState.STOW);
                    break;
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.POST_SCORING) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case LOB_PASS_PREP:
                arm.setLobPassPosition();
                flywheel.runFlywheelLobPass();
                hopper.stopHopper();
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASSING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case LOB_PASSING:
                if (!timer.hasElapsed(ScoringConstants.kShootingStateTime)) { // stop this once the piece is scored
                    flywheel.runFlywheelLobPass();
                    hopper.feedFlywheelLobPass();
                    intake.stopIntake();
                    timer.start();
                } else if (!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    Logger.getInstance().logEvent("Lob pass finished, arm angle " + arm.getArmAngleDegrees(), false);
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.reset();
                    requestState(SuperstructureState.GROUND_INTAKE);
                    break;
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.POST_SCORING) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case FRONT_LAYUP_PREP:
                arm.setFrontLayupPosition();
                flywheel.runFlywheelLayup();
                hopper.stopHopper(); // only when we are shooting in the shooting states do we run the hopper
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LAYUP_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case SIDE_LAYUP_PREP:
                arm.setSideLayupPosition();
                flywheel.runFlywheelLayup();
                hopper.stopHopper(); // only when we are shooting in the shooting states do we run the hopper
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LAYUP_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case LAYUP_SCORING:
                if (!timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    flywheel.runFlywheelLayup();
                    hopper.feedFlywheelLayup();
                    intake.stopIntake();
                    timer.start();
                } else if (!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    Logger.getInstance().logEvent("Layup shot finished, arm angle " + arm.getArmAngleDegrees(), false);
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.reset();
                    if (!DriverStation.isAutonomous()) {
                        drivetrain.setIsForcingCalibration(true);
                        lights.requestState(LightState.IDLE);
                        requestState(SuperstructureState.STOW);
                    }
                    break;
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case LL_PREP:
                arm.setLLPosition();
                flywheel.runFlywheelLimelight();

                if (hopper.getTopSensor()) {
                    hopper.stopHopper(); // only when we are shooting in the shooting states do we run the hopper
                } else {
                    hopper.setHopper(0.25);
                }

                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                }
                // removed conditions for gamepiece to be indexed and for arm to be at the right
                // angle
                // Look into this, for now just make sure flywheel is at the right RPM
                else if (requestedSystemState == SuperstructureState.LL_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case LL_SCORING:
                if (!timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    flywheel.runFlywheelLimelight();
                    hopper.feedFlywheelSpeaker();
                    intake.stopIntake();
                    timer.start();
                } else if (!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)) {
                    Logger.getInstance()
                            .logEvent("LL shot finished, distance " + LimelightShooter.getInstance().getDistance() +
                                    ", arm angle " + arm.getArmAngleDegrees(), false);
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.reset();
                    if (!DriverStation.isAutonomous()) {
                        drivetrain.setIsForcingCalibration(true);
                        lights.requestState(LightState.IDLE);
                        requestState(SuperstructureState.STOW);
                    }
                    break;
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;
            case CUSTOM_SHOT_PREP:
                arm.setArmAngle(customShotAngle);
                flywheel.runFlywheelLimelight();
                hopper.stopHopper(); // only when we are shooting in the shooting states do we run the hopper
                intake.stopIntake();

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                }
                // removed conditions for gamepiece to be indexed and for arm to be at the right
                // angle
                // Look into this, for now just make sure flywheel is at the right RPM
                else if (requestedSystemState == SuperstructureState.LL_SCORING && flywheel.isAtRPM()) {
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                }
                break;

            case POST_SCORING:
                arm.setStowPosition();
                flywheel.runFlywheelAmp();
                hopper.stopHopper();
                intake.stopIntake();

                if (arm.isAtStowAngle()) {
                    lights.requestState(LightState.IDLE);
                    requestState(SuperstructureState.STOW);
                }

                if (requestedSystemState == SuperstructureState.STOW) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.AMP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.FRONT_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.SIDE_LAYUP_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.PODIUM_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LOB_PASS_PREP) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.GROUND_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.OUTTAKE) {
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CUSTOM_SHOT_PREP) {
                    nextSystemState = requestedSystemState;
                }
                break;
        }

        systemState = nextSystemState;

    }

    public String stateAsString() { // possibly? using this to print state on shuffleboard
        switch (systemState) {
            case STOW:
                return "STOW";
            case GROUND_INTAKE:
                return "GROUND_INTAKE";
            case HP_INTAKE:
                return "HP_INTAKE";
            case AMP_PREP:
                return "AMP_PREP";
            case AMP_SCORING:
                return "AMP_SCORING";
            case FRONT_LAYUP_PREP:
                return "FRONT_LAYUP_PREP";
            case SIDE_LAYUP_PREP:
                return "SIDE_LAYUP_PREP";
            case LAYUP_SCORING:
                return "LAYUP_SCORING";
            case LL_PREP:
                return "LL_PREP";
            case LL_SCORING:
                return "LL_SCORING";
            case PODIUM_PREP:
                return "PODIUM_PREP";
            case PODIUM_SCORING:
                return "PODIUM SCORING";
            case LOB_PASS_PREP:
                return "LOB_PASS_PREP";
            case LOB_PASSING:
                return "LOB PASSING";
        }
        return "";
    }

    public void sendToScore() {
        if (systemState == SuperstructureState.AMP_PREP) {
            requestState(SuperstructureState.AMP_SCORING);
        } else if (systemState == SuperstructureState.LL_PREP) {
            requestState(SuperstructureState.LL_SCORING);
        } else if (systemState == SuperstructureState.FRONT_LAYUP_PREP) {
            requestState(SuperstructureState.LAYUP_SCORING);
        } else if (systemState == SuperstructureState.SIDE_LAYUP_PREP) {
            requestState(SuperstructureState.LAYUP_SCORING);
        } else if (systemState == SuperstructureState.PODIUM_PREP) {
            requestState(SuperstructureState.PODIUM_SCORING);
        } else if (systemState == SuperstructureState.LOB_PASS_PREP) {
            requestState(SuperstructureState.LOB_PASSING);
        }
    }

    // private boolean hasGamepiece(){ //this has potential use cases if we want to
    // keep a note in the intake instead of indexing it right away
    // return ((intake.hasGamepiece() || hopper.hasGamepiece()) ||
    // hasGamepieceOverride);
    // }

    private boolean isGamepieceIndexed() {
        if (SmartDashboard.getBoolean("Piece Indexed Override", false)) {
            return true;
        }
        return (hopper.isGamepieceIndexed() || isIndexedOverride);
    }

    public boolean isPassing() {
        return (systemState == SuperstructureState.LOB_PASS_PREP || systemState == SuperstructureState.LOB_PASSING);
    }

    public void setCustomShotAngle(double angle) {
        customShotAngle = angle;
    }

}