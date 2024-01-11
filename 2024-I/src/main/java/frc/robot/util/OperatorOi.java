package frc.robot.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorOi {
    private static OperatorOi instance;

    private PS4Controller Controller;

   
    
    public static OperatorOi getInstance(){
        if (instance == null){
            instance = new OperatorOi();
        }
        return instance;
    }

    public OperatorOi(){
        configureController();

    }
    public void configureController(){
        Controller = new PS4Controller(1);
        Trigger xbutton = new JoystickButton(Controller, 0);
    }

}

