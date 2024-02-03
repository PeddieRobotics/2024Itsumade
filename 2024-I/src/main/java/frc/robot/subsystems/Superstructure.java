package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Logger;
import frc.robot.utils.Constants.IntakeConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    // private final Arm arm;
    // private final Flywheel flywheel;
    private final Intake intake;
    // private final Hopper hopper;
    // private final Limelight limelight;
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
        // arm = Arm.getInstance();
        // flywheel = Flywheel.getInstance();
        intake = Intake.getInstance();
        // hopper = Hopper.getInstance();
        

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
                intake.stopIntake();
                if(requestedSystemState != SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } 
                break;   
            case GROUND_INTAKE:
                intake.setIntake(IntakeConstants.kIntakeSpeed);

                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case HP_INTAKE:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case AMP_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_SCORING){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case AMP_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LAYUP_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_SCORING){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LAYUP_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
                    nextSystemState = requestedSystemState; 
                }
                break;
            case LL_PREP:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LL_SCORING){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.AMP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.LAYUP_PREP){
                    //check if intake has gamepiece
                    nextSystemState = requestedSystemState; 
                }
                break; 
            case LL_SCORING:
                if(requestedSystemState == SuperstructureState.STOW){
                    nextSystemState = requestedSystemState;
                } else if(requestedSystemState == SuperstructureState.GROUND_INTAKE){
                    nextSystemState = requestedSystemState; 
                } else if(requestedSystemState == SuperstructureState.HP_INTAKE){
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