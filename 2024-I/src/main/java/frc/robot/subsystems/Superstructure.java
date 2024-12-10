package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;
    private SuperstructureState currentState;
    private SuperstructureState nextState;
    private Intake intake;
    private Cartridge cartridge;
    private Flywheel flywheel;

    public Superstructure(){
         intake = Intake.getInstance();  
         flywheel = Flywheel.getInstance();
         cartridge = Cartridge.getInstance();
         currentState = SuperstructureState.STOW;
         nextState = SuperstructureState.STOW;
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
                if(intake.getSensor()){
                    intake.setSpeed(0);
                }else{
                intake.setSpeed(0.3);
                }
                flywheel.runBlackFlywheelVelocitySetpoint(0);
                flywheel.runOrangeFlywheelVelocitySetpoint(0);
                cartridge.runHopper(0);
                break;

            case SHOOTING:
                // if(cartridge.getLowerSensor() || cartridge.getUpperSensor()){
                //  flywheel.runBlackFlywheelVelocitySetpoint(3000);
                //  flywheel.runOrangeFlywheelVelocitySetpoint(3000);
                //  cartridge.runHopper(
                //     0.3);
                // }else{
                flywheel.runBlackFlywheelVelocitySetpoint(3000);
                flywheel.runOrangeFlywheelVelocitySetpoint(3000);
                cartridge.runHopper(0.3);
                //}
                intake.setSpeed(0);
                break;

            case AMP:
            break;

            case STOW:
                intake.setSpeed(0);
                flywheel.runBlackFlywheelVelocitySetpoint(0);
                flywheel.runOrangeFlywheelVelocitySetpoint(0);
                cartridge.runHopper(0);
                break;

        }

        currentState = nextState;

    }
    


}
