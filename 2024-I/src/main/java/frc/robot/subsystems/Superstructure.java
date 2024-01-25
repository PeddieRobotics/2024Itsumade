package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Logger;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private Arm arm;
    private Flywheel flywheel;
    private double stateDuration;
    private double internalStateTimer;

    public enum SuperstructureState{
        NEUTRAL,
        GROUND_INTAKE,
        HP_INTAKE,
        SHOOTING,
        AMP,
        CLIMBING,
    }

    SuperstructureState systemState;
    SuperstructureState requestedSystemState;

    public Superstructure(){
        arm = Arm.getInstance();
        flywheel = Flywheel.getInstance();
        

        systemState = SuperstructureState.NEUTRAL;
        requestedSystemState = SuperstructureState.NEUTRAL;

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
            case NEUTRAL:

                if(requestedSystemState != SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break;   
            case GROUND_INTAKE:
                if(requestedSystemState == SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case HP_INTAKE:
                if(requestedSystemState == SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case SHOOTING:
                if(requestedSystemState == SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case AMP:
                if(requestedSystemState == SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break; 
            case CLIMBING:
                if(requestedSystemState == SuperstructureState.NEUTRAL){
                    nextSystemState = requestedSystemState;
                }
                break; 
        }
        systemState = nextSystemState;
    }
}