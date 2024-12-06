package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;

public class Cartridge extends SubsystemBase{
    
    public static Cartridge cartridge;
    private Kraken motor;

    public Cartridge(){
        motor = new Kraken(Constants.CartridgeConstants.cartridge_id, Constants.GlobalConstants.canivore_name);

        motor.setSupplyCurrentLimit(Constants.CartridgeConstants.cartridge_current_limit);

        SmartDashboard.putNumber("cartridge setpoint", 0);
    }

    public void runHopper(double speed){
        motor.setMotor(speed);
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
        runHopper(SmartDashboard.getNumber("cartridge setpoint", 0));
       
        SmartDashboard.putNumber("cartridge rpm", motor.getRPM());
    }
}
