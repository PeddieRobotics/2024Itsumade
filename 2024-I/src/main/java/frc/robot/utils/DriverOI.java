package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbCommands.RetractClimber;
import frc.robot.commands.DriveCommands.FollowNote;
import frc.robot.commands.DriveCommands.FollowNoteInAuto;
import frc.robot.commands.DriveCommands.ForcedCalibration;
import frc.robot.commands.DriveCommands.PathPlannerToPoint;
import frc.robot.commands.DriveCommands.PathPlannerToShoot;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.Target;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
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

    private AlignGoalColumn alignGoalColumn = AlignGoalColumn.kCenter;

    private Superstructure superstructure;
    private Drivetrain drivetrain;

    public DriverOI() {
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();
        configureController();
    }

    public int getAlignGoalAprilTagID() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
    }

    public AlignGoalColumn getAlignGoalColumn() {
        return alignGoalColumn;
    }

    public void configureController() {
        controller = new PS4Controller(0);

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.GROUND_INTAKE)));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.PODIUM_PREP)));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> superstructure.sendToScore()));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.whileTrue(new FollowNote());

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.OUTTAKE)));

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.whileTrue(new Target());

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.onTrue(new RetractClimber());

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

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

    public double getForward() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(input) < 0.9) {
            input *= 0.7777;
        } else {
            input = Math.pow(input, 3);
        }
        return input;
    }

    public double getRightForward(){
        double input = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
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