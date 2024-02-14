package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

public class DriverOI {
    public enum AlignGoalColumn {
        kCenter, kLeft, kRight
    }

    private static DriverOI instance;

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }

        return instance;
    }

    private PS4Controller controller;

    /**
     * the center depending on aliance
     */
    private int alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
    private AlignGoalColumn alignGoalColumn = AlignGoalColumn.kCenter;

    private Arm arm;
    private Superstructure superstructure;
    private Drivetrain drivetrain;

    private boolean usePreScorePose;

    private Trigger xButton, touchpadButton, circleButton, triangleButton, muteButton, squareButton, L1Bumper, R1Bumper;

    public DriverOI() {
        arm = Arm.getInstance();
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();
        configureController(false);
    }

    public int getAlignGoalAprilTagID() {
        return alignGoalAprilTagID;
    }

    public AlignGoalColumn getAlignGoalColumn() {
        return alignGoalColumn;
    }

    public void controlLoop() {
        if (xButton.getAsBoolean()) {
            superstructure.requestState(SuperstructureState.GROUND_INTAKE);
        } else if (muteButton.getAsBoolean()) {
            superstructure.requestState(SuperstructureState.STOW);
        } else if (circleButton.getAsBoolean()) {
            superstructure.requestState(SuperstructureState.HP_INTAKE);
        } else if (triangleButton.getAsBoolean()) {
            superstructure.requestState(SuperstructureState.LL_SCORING); // CHANGE THIS LATER BECAUSE THERE IS DEEPER
                                                                         // LOGIC REQUIRED
        } else if (squareButton.getAsBoolean()) {
            // drive to note command here
        } else if (L1Bumper.getAsBoolean()) {
            // align command here
        } else if (R1Bumper.getAsBoolean()) {
            superstructure.requestState(SuperstructureState.CLIMBING);
        }
    }

    public void configureController(boolean usePreScorePose) {
        controller = new PS4Controller(0);

        // Arm Poses
        // L1 score (will move to this pose regardless of having a gamepiece)
        xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);

        // L2 scoring pose
        circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);

        // L3 scoring pose - does not include L3 cone forward right now.
        triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);

        // Square button forces the robot to look at odometry updates.
        squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);

        // Stowed pose
        touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);

        // Mute homes the entire arm subsystem, both wrist and shoulder.
        muteButton = new JoystickButton(controller, 15);

        // Manual Wrist and Shoulder Override Controls
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));

        // Press and hold for outtaking slow (gamepiece adjustment), with down arrow
        // this becomes full speed.
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        // Press and hold for intaking slow (gamepiece adjustment), with down arrow this
        // becomes full speed.
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

        // Game piece selection / LED indication requests to human player
        L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);

        R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);

        // Column Selection
        Trigger dpadUpTrigger = new Trigger(() -> controller.getPOV() == 0);

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);

        Trigger dpadDownTrigger = new Trigger(() -> controller.getPOV() == 180);

    }

    private boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    private boolean bothTriggersHeld() {
        return leftTriggerHeld() & rightTriggerHeld();
    }

    private boolean leftTriggerHeld() {
        return controller.getL2Button();
    }

    private boolean onlyLeftTriggerHeld() {
        return leftTriggerHeld() && !rightTriggerHeld();
    }

    private boolean rightTriggerHeld() {
        return controller.getR2Button();
    }

    private boolean onlyRightTriggerHeld() {
        return !leftTriggerHeld() && rightTriggerHeld();
    }

    private boolean onlyOneTriggerHeld() {
        return leftTriggerHeld() ^ rightTriggerHeld();
    }

    public boolean dPadDownHeld() {
        return controller.getPOV() == 180;
    }

    public boolean isUsePreScorePose() {
        return usePreScorePose;
    }

    // Only update the boolean for using the pre-score pose if it is a state change
    // This is especially important since this requires configuring the controller
    // mapping
    // for the operator, which should be done infrequently/minimally.
    public void setUsePreScorePose(boolean usePreScorePose) {
        if (this.usePreScorePose != usePreScorePose) {
            this.usePreScorePose = usePreScorePose;
            configureController(usePreScorePose);
        }
    }

    public void configureController() {
        controller = new PS4Controller(0);
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
    }

    public double getForward() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(input) < 0.9) {
            input *= 0.7777;
        } else {
            input = Math.pow(input, 3);
        }
        return input;
    }

    public double getStrafe() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        if (Math.abs(input) < 0.9) {
            input *= 0.7777;
        } else {
            input = Math.pow(input, 3);
        }
        return input;
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < OIConstants.kDrivingDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, OIConstants.kDrivingDeadband);

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation;
        combinedRotation = (leftRotation - rightRotation) / 2.0;

        return combinedRotation * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = controller.getRawAxis(2) * DriveConstants.kWheelBase;
        double rotY = controller.getRawAxis(5) * DriveConstants.kTrackWidth;

        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

}