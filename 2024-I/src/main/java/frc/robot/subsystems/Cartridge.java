package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;

public class Cartridge extends SubsystemBase{
    
    public static Cartridge cartridge;
    private Kraken motor;
    private DigitalInput lowerSensor;
    private DigitalInput upperSensor;

    public Cartridge(){
        motor = new Kraken(Constants.CartridgeConstants.cartridge_id, Constants.GlobalConstants.canivore_name);
        lowerSensor = new DigitalInput(Constants.CartridgeConstants.cartridge_lower_sensor);
        upperSensor = new DigitalInput(Constants.CartridgeConstants.cartridge_upper_sensor);

        motor.setSupplyCurrentLimit(Constants.CartridgeConstants.cartridge_current_limit);

        SmartDashboard.putNumber("cartridge setpoint", 0);
    }

    public void runHopper(double speed){
        motor.setMotor(speed);
    }

    public boolean getLowerSensor(){
        return !lowerSensor.get();
    }

    public boolean getUpperSensor(){
        return !upperSensor.get();
    }

    public static Cartridge getInstance(){
        if(cartridge == null){
            cartridge = new Cartridge();
        }
        return cartridge;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      // intakeMotor.set(TalonSRXControlMode.PercentOutput,SmartDashboard.getNumber("Intake
      // speed", 0));
        //runHopper(SmartDashboard.getNumber("cartridge setpoint", 0));
       
        SmartDashboard.putNumber("cartridge rpm", motor.getRPM());
    }
}
