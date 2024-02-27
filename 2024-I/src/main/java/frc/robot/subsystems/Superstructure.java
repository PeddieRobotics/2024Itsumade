package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.ScoringConstants;

import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private final Arm arm;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Hopper hopper;
    private double stateDuration;
    private double internalStateTimer;
    private double shootingSpeed;
    private boolean isIndexedOverride, hasGamepieceOverride, justIntaked;
    private Timer timer;

    public enum SuperstructureState {
        LL_TEST,
        STOW,
        GROUND_INTAKE,
        HP_INTAKE,
        AMP_PREP,
        AMP_SCORING,
        LAYUP_PREP,
        LAYUP_SCORING,
        LL_PREP,
        LL_SCORING
    }

    SuperstructureState systemState;
    SuperstructureState nextSystemState;
    SuperstructureState requestedSystemState;

    public Superstructure() {
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        timer = new Timer();

        systemState = SuperstructureState.STOW;
        nextSystemState = SuperstructureState.STOW;
        requestedSystemState = SuperstructureState.STOW;
        isIndexedOverride = false;

        SmartDashboard.putBoolean("Piece Indexed Override", isIndexedOverride); // overrides, just in case
        // hasGamepieceOverride = SmartDashboard.putBoolean("Has Gamepiece Override",
        // false);

        SmartDashboard.putString("STATE", systemState.toString());
        stateDuration = 0;
        shootingSpeed = 0;
        internalStateTimer = 0;

        SmartDashboard.putBoolean("LL Shot Move Arm", false);
        SmartDashboard.putNumber("LL Shot Angle", 0);
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

    public void shoot(double flywheelspeed) {
        requestedSystemState = SuperstructureState.LAYUP_PREP;
        shootingSpeed = flywheelspeed;
    }

    public String getRobotState() {
        return systemState.toString();
    }

    @Override
    public void periodic(){
        arm.setState(systemState.toString());

        switch(systemState){
            case LL_TEST:
                if(SmartDashboard.getBoolean("LL Shot Move Arm", false)){
                    arm.setArmAngle(SmartDashboard.getNumber("LL Shot Angle", 0));
                }
                flywheel.runFlywheelLimelight();

                if(flywheel.isAtRPM()){
                    hopper.feedFlywheelSpeaker();
                }

                intake.stopIntake();

                if(requestedSystemState == SuperstructureState.STOW){ 
                    nextSystemState = requestedSystemState; 
                }
                break;


            //idle state of robot, arm is in stow position, 
            case STOW:
                SmartDashboard.putBoolean("ARM At Stow Angle", arm.isAtStowAngle());
                if(arm.isAtStowAngle()){
                    SmartDashboard.putBoolean("Neutral Stow", true);
                    arm.setArmNeutralMode();
                } else {
                    SmartDashboard.putBoolean("Neutral Stow", false);
                    arm.setStowPosition();
                }

                flywheel.stopFlywheel();
                intake.stopIntake();
                hopper.stopHopper();

                if(requestedSystemState == SuperstructureState.LL_TEST){ 
                    nextSystemState = requestedSystemState; 
                }

                 //only go into the prep state if you have a gamepiece
                if(requestedSystemState == SuperstructureState.AMP_PREP){ 
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState;
                } 
                
                 //intake only if you don't have a piece -- ASSUMING WE'RE INDEXING RIGHT AWAY
                else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){ 
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                } 

                 //you should not be able to go into scoring from stow **someone please verify if this makes sense
                // else if(requestedSystemState == SuperstructureState.AMP_SCORING && isGamepieceIndexed()){
                //     nextSystemState = requestedSystemState;
                // } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING && isGamepieceIndexed()){
                //     nextSystemState = requestedSystemState;
                // } else if (requestedSystemState == SuperstructureState.LL_SCORING && isGamepieceIndexed()){
                //     nextSystemState = requestedSystemState;
                // } 
                
                break;   

            case GROUND_INTAKE:
                flywheel.stopFlywheel();
                arm.setGroundIntakePosition();
                intake.runIntake();
                hopper.runHopperGroundIntake();
                if (isGamepieceIndexed()){
                    intake.stopIntake();
                    hopper.stopHopper();
                    requestState(SuperstructureState.STOW);
                }

                //only switch states from here if done indexing, but let it go into stow from anywhere
                if(requestedSystemState == SuperstructureState.STOW){ //CHECK THIS
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE && !isGamepieceIndexed()){
                    nextSystemState = requestedSystemState; 
                } 

                break; 

            case HP_INTAKE:
                if(!hopper.getTopSensor() || !hopper.getBottomSensor()){
                    arm.setHPIntakePosition();
                    flywheel.runFlywheelHP();
                    intake.stopIntake();
                } else if(hopper.getBottomSensor()) {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    requestState(SuperstructureState.STOW);
                    break;
                }

                //only switch states from intake if done indexing, but let it go into stow from anywhere
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } 
                break; 

            case AMP_PREP:
                arm.setAmpPosition();
                flywheel.runFlywheelAmp();
                hopper.stopHopper();
                intake.stopIntake();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_SCORING && flywheel.isAtRPM()){
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 


            case AMP_SCORING:
                if(!timer.hasElapsed(ScoringConstants.kShootingStateTime)){ //stop this once the piece is scored
                    flywheel.runFlywheelAmp();
                    hopper.feedFlywheelAmp();
                    intake.stopIntake();
                    timer.start();
                } else if(!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)){
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.stop();
                    timer.reset();
                    setBackToStowAfterShot(); //requestState(SuperstructureState.STOW);
                    break;
                }

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case LAYUP_PREP:
                arm.setLayupPosition();
                flywheel.runFlywheelLayup();
                hopper.stopHopper(); //only when we are shooting in the shooting states do we run the hopper
                intake.stopIntake();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING && flywheel.isAtRPM()){
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case LAYUP_SCORING:
                if(!timer.hasElapsed(ScoringConstants.kShootingStateTime)){
                    flywheel.runFlywheelLayup();
                    hopper.feedFlywheelLayup();
                    intake.stopIntake();
                    timer.start();
                } else if(!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)){
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.stop();
                    timer.reset();
                    setBackToStowAfterShot(); //requestState(SuperstructureState.STOW);
                    break;
                }

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break;

            case LL_PREP:
                arm.setLLPosition();
                flywheel.runFlywheelLimelight();
                hopper.stopHopper(); //only when we are shooting in the shooting states do we run the hopper
                intake.stopIntake();

                
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } 
                // removed conditions for gamepiece to be indexed and for arm to be at the right angle
                // Look into this, for now just make sure flywheel is at the right RPM
                else if(requestedSystemState == SuperstructureState.LL_SCORING && flywheel.isAtRPM()){
                    timer.reset();
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case LL_SCORING:
                if(!timer.hasElapsed(ScoringConstants.kShootingStateTime)){
                    flywheel.runFlywheelLimelight();
                    hopper.feedFlywheelSpeaker();
                    intake.stopIntake();
                    timer.start();
                } else if(!isGamepieceIndexed() && timer.hasElapsed(ScoringConstants.kShootingStateTime)){
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                    timer.stop();
                    timer.reset();
                    setBackToStowAfterShot(); //requestState(SuperstructureState.STOW);
                    break;
                }

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break;

            
        }

    if(nextSystemState!=systemState)justIntaked=false;systemState=nextSystemState;

    }

    public String stateAsString(){ //possibly? using this to print state on shuffleboard
        switch(systemState){
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
            case LAYUP_PREP:
                return "LAYUP_PREP";
            case LAYUP_SCORING:
                return "LAYUP_SCORING";
            case LL_PREP:
                return "LL_PREP"; 
            case LL_SCORING:
                return "LL_SCORING";
        }
        return "";
    }

    public void sendToScore(){
        if(systemState == SuperstructureState.AMP_PREP){
            requestState(SuperstructureState.AMP_SCORING);
        } else if(systemState == SuperstructureState.LL_PREP){
            requestState(SuperstructureState.LL_SCORING);
        } else if(systemState == SuperstructureState.LAYUP_PREP){
            requestState(SuperstructureState.LAYUP_SCORING);
        }
    }

    // private boolean hasGamepiece(){ //this has potential use cases if we want to
    // keep a note in the intake instead of indexing it right away
    // return ((intake.hasGamepiece() || hopper.hasGamepiece()) ||
    // hasGamepieceOverride);
    // }

    private boolean isGamepieceIndexed() {
        if(SmartDashboard.getBoolean("Piece Indexed Override", false)){
        return true;
        }
        return (hopper.isGamepieceIndexed() || isIndexedOverride);
    }

    private boolean stowAfterShot(){
        return SmartDashboard.getBoolean("Stow After Shoot Override", true);
    }

    public void setBackToStowAfterShot(){
        if(stowAfterShot()){
            requestState(SuperstructureState.STOW);
        }
    }
}