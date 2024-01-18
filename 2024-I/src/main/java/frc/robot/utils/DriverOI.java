package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverOI{

    private PS4Controller controller;
    private static DriverOI driverOI;

    public DriverOI(){

    }

    public static DriverOI getInstance(){
        if(driverOI == null){
            driverOI = new DriverOI();
        }return driverOI;
    }

    public void configureController(){
        controller = new PS4Controller(0);
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
    }
}