package frc.robot.subsystems;

public class Flywheel {

    private static Flywheel instance;
    public enum ArmState{Intaking, Moving, Stowed, Shooting}

    private ArmState state, goalState;


    public static Flywheel getInstance(){
        if (instance == null){
            instance = new Flywheel();
        }
        return instance;
    }


    public Flywheel(){
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
