package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RunFlywheel;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class OI {
    
    private static OI oi;
    private PS4Controller controller;
    private Superstructure superstructure;

    public OI(){
        superstructure = Superstructure.getInstance();
        controller = new PS4Controller(0);
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        Trigger oButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.INTAKING)));
        oButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.SHOOTING)));
        touchpadButton.onTrue(new InstantCommand(()-> superstructure.requestState(SuperstructureState.STOW)));
       
    }

    public static OI getInstance(){
        if(oi == null){
            
            oi = new OI();
        }
        return oi;
    }
}
