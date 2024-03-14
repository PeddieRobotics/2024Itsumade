package frc.robot.utils;

import javax.security.auth.kerberos.DelegationPermission;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ClimbCommands.DeployClimber;
import frc.robot.commands.ClimbCommands.ManualClimberControl;
import frc.robot.commands.ClimbCommands.RetractClimber;
import frc.robot.commands.DriveCommands.Target;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.subsystems.Superstructure;

public class OperatorOI {
    private static OperatorOI instance;

    public static OperatorOI getInstance() {
        if (instance == null) {
            instance = new OperatorOI();
        }
        return instance;
    }

    private PS4Controller controller;

    /**
     * the center depending on aliance
     */

    private Drivetrain drivetrain;
    private Superstructure superstructure;

    public OperatorOI() {
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        configureController();
    }

    public int getAlignGoalAprilTagID() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
    }

    public void configureController() {
        controller = new PS4Controller(1);

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.PODIUM_PREP)));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.AMP_PREP)));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> {
            if (rightTriggerHeld()) {
                superstructure.requestState(SuperstructureState.SIDE_LAYUP_PREP);
            } else {
                superstructure.requestState(SuperstructureState.FRONT_LAYUP_PREP);
            }
        }));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.LL_PREP)));

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger muteButton = new JoystickButton(controller, 15);

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new DeployClimber());

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.onTrue(new RetractClimber());

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        L2Trigger.whileTrue(new ManualClimberControl());

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        ps5Button.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.OUTTAKE)));

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.onTrue(new InstantCommand(() -> DriverOI.getInstance().toggleUseOdometryTarget()));

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

    public double getLeftForward() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(input) < OIConstants.kDrivingDeadband) {
            input = 0;
        } else {
            input *= 0.7777;
        }
        return input;
    }

    public double getRightForward() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        if (Math.abs(input) < OIConstants.kDrivingDeadband) {
            input = 0;
        } else {
            input *= 0.7777;
        }
        return input;
    }

}