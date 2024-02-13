package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private final Arm arm;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Hopper hopper;
    private double stateDuration;
    private double internalStateTimer;
    private double shootingSpeed;

    public enum SuperstructureState{
        STOW,
        GROUND_INTAKE,
        HP_INTAKE,
        AMP_PREP,
        AMP_SCORING,
        LAYUP_PREP,
        LAYUP_SCORING,
        LL_PREP,
        LL_SCORING,
        CLIMBING
    }

    SuperstructureState systemState;
    SuperstructureState nextSystemState;
    SuperstructureState requestedSystemState;

    public Superstructure(){
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        

        systemState = SuperstructureState.STOW;
        requestedSystemState = SuperstructureState.STOW;

        stateDuration = 0;
        shootingSpeed=0;
        internalStateTimer = 0;
    }

    public static Superstructure getInstance() {
        if (superstructure == null) {
            superstructure = new Superstructure();
        }
        return superstructure;
    }

    public void requestState(SuperstructureState request){
        requestedSystemState = request;
    }

    public void shoot(double flywheelspeed){
        requestedSystemState=SuperstructureState.LAYUP_PREP;
        shootingSpeed=flywheelspeed;

    }

    public String getRobotState(){
        return (systemState.toString());
    }

    @Override
    public void periodic(){
        nextSystemState = systemState;

        switch(systemState){

            //idle state of robot, arm is in stow position, 
            case STOW:
                arm.setStowPosition();
                intake.stopIntake();
                hopper.stopHopper();
                
                if(intake.hasGamepiece()){
                    //let the note transition from intake to hopper, bensalem thing
                } else {
                    hopper.stopHopper();
                }

                if(requestedSystemState == SuperstructureState.AMP_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE && !hasGamepiece()){ 
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE && !hasGamepiece()){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.AMP_SCORING && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.LL_SCORING && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if (requestedSystemState == SuperstructureState.CLIMBING){
                    nextSystemState = requestedSystemState;
                }
                break;   

            case GROUND_INTAKE:
                arm.setGroundIntakePosition();
                if (arm.isAtAngle()){
                    intake.runIntake();
                    hopper.runHopper();
                } else if(!arm.isAtAngle() || isGamepieceIndexed()) {
                    intake.stopIntake();
                    hopper.stopHopper();
                }

                if(requestedSystemState == SuperstructureState.STOW || hasGamepiece()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case HP_INTAKE:
                if(!isGamepieceIndexed()){
                    arm.setHPIntakePosition();
                    flywheel.runFlywheelHP();
                    hopper.runHopperHP();
                    intake.stopIntake();
                } else {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                }

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case AMP_PREP:
                if(isGamepieceIndexed()){
                    arm.setAmpPosition();
                    flywheel.runFlywheelAmp();
                    hopper.stopHopper(); //only when we are shooting in the shooting states do we run the hopper
                    intake.stopIntake();
                } else {}
                
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_SCORING){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case AMP_SCORING:
                if(isGamepieceIndexed()){
                    flywheel.runFlywheelAmp();
                    hopper.runHopper();
                    intake.stopIntake();
                } else {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
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
                if(isGamepieceIndexed()){
                    arm.setLayupPosition();
                    flywheel.runFlywheelShot();
                    hopper.stopHopper(); //only when we are shooting in the shooting states do we run the hopper
                    intake.stopIntake();
                } else {}

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case LAYUP_SCORING:
                if(isGamepieceIndexed()){
                    flywheel.runFlywheelShot();
                    hopper.runHopper();
                    intake.stopIntake();
                } else {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
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
                if(isGamepieceIndexed()){
                    arm.setLLPosition();
                    flywheel.runFlywheelShot();
                    hopper.stopHopper(); //only when we are shooting in the shooting states do we run the hopper
                    intake.stopIntake();
                } else {}

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_SCORING && isGamepieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 

            case LL_SCORING:
                if(isGamepieceIndexed()){
                    flywheel.runFlywheelShot();
                    hopper.runHopper();
                    intake.stopIntake();
                } else {
                    flywheel.stopFlywheel();
                    hopper.stopHopper();
                }

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break;

            case CLIMBING:
                arm.setStowPosition();
                flywheel.stopFlywheel();
                hopper.stopHopper();
                intake.stopIntake();
                //Climbing logic for the state here; nothing is in the subsystem so can't do anything
                
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
        }
        systemState = nextSystemState;
        //SmartDashboard.putString("State", stateAsString);
    }

    public String stateAsString(){
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
            case CLIMBING:
                return "CLIMBING";
        }
        return "";
    }

    private boolean hasGamepiece(){
        if(hopper.hasGamepiece() || intake.hasGamepiece()) {
            return true;
        } 
        return false;
    }

    private boolean isGamepieceIndexed() {
        if(!intake.hasGamepiece() && hopper.isGamepieceIndexed()){
            return true;
        }
        return false;
    }
}