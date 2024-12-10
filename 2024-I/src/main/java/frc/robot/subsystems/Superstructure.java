package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private SuperstructureState currentState;
    private SuperstructureState nextState;
    private Intake intake;
    private Flywheel flywheel;

    public Superstructure(){
         intake = Intake.getInstance();  
         flywheel = Flywheel.getInstance();
    }

    public enum SuperstructureState{
        INTAKING,
        SHOOTING,
        AMP,
        STOW
    }

    public void requestState(SuperstructureState requestedState){
        nextState = requestedState;
    }

    public static Superstructure getInstance(){
        if(superstructure == null){
            superstructure = new Superstructure();
        }
        return superstructure;
    }

   
    @Override
    public void periodic(){
        switch(currentState){
            case INTAKING:
                intake.setSpeed(0.3);

            case SHOOTING:
                flywheel.runBlackFlywheelVelocitySetpoint(300);
                flywheel.runOrangeFlywheelVelocitySetpoint(300);

            case AMP:

            case STOW:
                intake.setSpeed(0);
                flywheel.runBlackFlywheelVelocitySetpoint(0);
                flywheel.runOrangeFlywheelVelocitySetpoint(0);

        }

    }


}
