package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RunFlywheel;

public class OI {
    
    private static OI oi;
    private PS4Controller controller;

    public OI(){
        controller = new PS4Controller(0);
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        Trigger oButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        xButton.whileTrue(new IntakeCommand());
        oButton.whileTrue(new RunFlywheel());
    }

    public static OI getInstance(){
        if(oi == null){
            oi = new OI();
        }
        return oi;
    }
}
