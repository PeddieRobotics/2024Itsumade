package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private Arm arm;
    private Flywheel flywheel;
    private Limelight limelight;
    private double stateDuration;
    private double internalStateTimer;

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
    SuperstructureState requestedSystemState;

    public Superstructure(){
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        

        systemState = SuperstructureState.STOW;
        requestedSystemState = SuperstructureState.STOW;

        stateDuration = 0;
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

    public String getRobotState(){
        return (systemState.toString());
    }

    @Override
    public void periodic(){
        SuperstructureState nextSystemState = systemState;

        switch(systemState){

            //idle state of robot, arm is in stow position
            case STOW:

                if(requestedSystemState != SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break;   
            case GROUND_INTAKE:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case HP_INTAKE:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case AMP_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case AMP_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case LAYUP_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case LAYUP_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case LL_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case LL_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case CLIMBING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                }
                break; 
        }
        systemState = nextSystemState;
    }
}