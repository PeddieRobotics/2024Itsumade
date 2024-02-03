package frc.robot.subsystems;

public class Arm {

    private static Arm instance;
    public enum ArmState{Intaking, Moving, Stowed, Shooting}

    private ArmState state, goalState;


    public static Arm getInstance(){
        if (instance == null){
            instance = new Arm();
        }
        return instance;
    }


    public Arm(){
        state = ArmState.Stowed;
        goalState = ArmState.Stowed;

    }

    public void RequestState(ArmState requestedState){
        if (requestedState == state || requestedState == goalState){
            return;
        }
        state = ArmState.Moving;
        goalState = requestedState;
    }

    public void periodic(){

    }
    
    
}
