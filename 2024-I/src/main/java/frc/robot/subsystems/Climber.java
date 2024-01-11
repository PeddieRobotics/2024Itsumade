package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private static Climber instance;
    private Solenoid climber;
    
    public static Climber getInstance(){
        if (instance == null){
            instance = new Climber();
        }
        return instance;
    }

    public Climber(){
        climber = new Solenoid(null,0); //temporary values
    }

    public void ExtendClimber(){
        climber.set(true);
    }

    public void RetractClimber(){
        climber.set(false);
    }

    public boolean GetClimberState(){
        return climber.get();
    }
}
