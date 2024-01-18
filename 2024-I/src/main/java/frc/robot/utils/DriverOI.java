package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;

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

    public double getForward() {
        double input = controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if(Math.abs(input) < 0.9){
            input *= 0.7777;
        }
        else{
            input = Math.pow(input, 3);
        }
        return input;
    }

    public double getStrafe() {
        double input = controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        if(Math.abs(input) < 0.9){
            input *= 0.7777;
        }
        else{
            input = Math.pow(input, 3);
        }
        return input;
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < OIConstants.kDrivingDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, OIConstants.kDrivingDeadband);

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation;
        combinedRotation = (rightRotation - leftRotation) / 2.0;

        return combinedRotation * DriveConstants.kMaxAngularSpeed;
    }

    public Translation2d getCenterOfRotation() {
        double rotX = controller.getRawAxis(2) * DriveConstants.kWheelBase;
        double rotY = controller.getRawAxis(5) * DriveConstants.kTrackWidth;

        if (rotX * rotY > 0) {
            rotX = -rotX;
            rotY = -rotY;
        }
        rotX *= 0.75;
        rotY *= 0.75;
        Translation2d output = new Translation2d(rotX, rotY);
        return output;
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }
}