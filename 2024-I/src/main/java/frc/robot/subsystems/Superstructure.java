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
                flywheel.stopFlywheel();
                intake.stopIntake();
                
                if(intake.hasGamepiece()){
                    //let the note transition from intake to hopper
                } else {
                    hopper.stopHopper();
                }

                if(requestedSystemState != SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } 
                break;   
            case GROUND_INTAKE:
                // arm.setIntakePosition();
                // flywheel.stopFlywheel();
                // if (arm.canIntake()) intake.setIntake(IntakeConstants.kIntakeSpeed);
                // else intake.stopIntake();
                // hopper.index();
                intake.setIntake(IntakeConstants.kIntakeSpeed);

                if(requestedSystemState == SuperstructureState.STOW || pieceIndexed()){
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
                intake.stopIntake();
                // arm.setHPIntakePosition();
                // flywheel.stopFlywheel();
                // hopper.hpIndex();

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
                intake.stopIntake();
                // arm.setAmpPosition();
                // flywheel.amp();
                // hopper.index();
                
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
                intake.stopIntake();
                // arm.setAmpPosition();
                // flywheel.amp();
                // hopper.feed();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LAYUP_PREP:
                intake.stopIntake();
                // arm.layup();
                // flywheel.layup();
                // hopper.index();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING && pieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LAYUP_SCORING:
                intake.stopIntake();
                // arm.layup();
                // flywheel.layup();
                // hopper.feed();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break;
            case LL_PREP:
                intake.stopIntake();
                // arm.llalign();
                // flywheel.llshoot();
                // hopper.index();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_SCORING && pieceIndexed()){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LL_SCORING:
                intake.stopIntake();
                // arm.llalign();
                // flywheel.llshoot();
                // hopper.feed();

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case CLIMBING:
                intake.stopIntake();
                // arm.setStowPosition();
                // flywheel.stopFlywheel();
                // hopper.stopHopper();
                
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

    private boolean robotHasPiece(){
        // return (intake.getSensor()||hopper.bottomSensor()||hopper.topSensor());
        return false;
    }

    private boolean pieceIndexed(){ //cartridge = hopper, I better catch none of you calling it something that goes on a printer
        // return (!intake.getSensor()&&hopper.bottomSensor()&&hopper.topSensor());
        return false;
    }

}