package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.ManualArmControl;
import frc.robot.subsystems.Arm;
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
    private int alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;

    private Arm arm;
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private Trigger xButton, circleButton, ps5Button, triangleButton, muteButton, squareButton, L1Bumper, R1Bumper, L2Trigger, R2Trigger;

    public OperatorOI() {
        arm = Arm.getInstance();
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();
        configureController();
    }

    public int getAlignGoalAprilTagID() {
        return alignGoalAprilTagID;
    }

    public void controlLoop(){
        if(xButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.STOW);
        } else if(circleButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.AMP_PREP);
        } else if(triangleButton.getAsBoolean()){
                superstructure.requestState(SuperstructureState.LAYUP_PREP); //CHANGE THIS LATER BECAUSE THERE IS DEEPER LOGIC REQUIRED
        } else if(squareButton.getAsBoolean()){
                superstructure.requestState(SuperstructureState.LL_PREP);
        } else if(L1Bumper.getAsBoolean()){
                //align command here
        } else if(R1Bumper.getAsBoolean()){
               superstructure.requestState(SuperstructureState.DEPLOY_CLIMBER);  
        } 

        ps5Button.onTrue(new InstantCommand(() -> drivetrain.resetGyro()));
        L2Trigger.whileTrue(new ManualArmControl());
    }

    public void configureController() {
        controller = new PS4Controller(1);

        // Arm Poses
        // L1 score (will move to this pose regardless of having a gamepiece)
        xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);

        // L2 scoring pose
        circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);

        // L3 scoring pose - does not include L3 cone forward right now.
        triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);

        // Square button forces the robot to look at odometry updates.
        squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);

        // Touchpad Button (if we need it)
        //touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);

        // Mute homes the entire arm subsystem, both wrist and shoulder.
        muteButton = new JoystickButton(controller, 15);

        // Manual Wrist and Shoulder Override Controls
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger LStick = new JoystickButton(controller, PS4Controller.Axis.kLeftY.value);

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);

        // Press and hold for outtaking slow (gamepiece adjustment), with down arrow this becomes full speed.
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        // Press and hold for intaking slow (gamepiece adjustment), with down arrow this becomes full speed.
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

    private boolean leftTriggerHeld(){
        return controller.getL2Button();
    }

    private boolean onlyLeftTriggerHeld(){
        return leftTriggerHeld() && !rightTriggerHeld();
    }

    private boolean rightTriggerHeld(){
        return controller.getR2Button();
    }

    private boolean onlyRightTriggerHeld(){
        return !leftTriggerHeld() && rightTriggerHeld();
    }

    private boolean onlyOneTriggerHeld() {
        return leftTriggerHeld() ^ rightTriggerHeld();
    }

    public boolean dPadDownHeld(){
        return controller.getPOV() == 180;
    }

    public double getForward() {
        double input = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(input) < OIConstants.kDrivingDeadband) {
            input = 0;
        } else {
            input *= 0.7777;
        }
        return input;
    }

    /*public boolean isUsePreScorePose() {
        return usePreScorePose;
    }

    // Only update the boolean for using the pre-score pose if it is a state change
    // This is especially important since this requires configuring the controller mapping
    // for the operator, which should be done infrequently/minimally.
    public void setUsePreScorePose(boolean usePreScorePose) {
        if(this.usePreScorePose != usePreScorePose){
            this.usePreScorePose = usePreScorePose;
            configureController(usePreScorePose);
        }
    }
    */

}