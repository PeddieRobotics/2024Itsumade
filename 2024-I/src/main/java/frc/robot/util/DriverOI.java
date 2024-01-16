package frc.robot.util;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverOI {
    private static DriverOI instance;
    private PS4Controller controller;

    public static DriverOI getInstance(){
        if (instance == null){
            instance = new DriverOI();
        }
        return instance;
    }

    public DriverOI(){
        configureController();
    }
    public void configureController(){
        controller = new PS4Controller(0);
        Trigger xbutton = new JoystickButton(controller, 0);
    }
}


