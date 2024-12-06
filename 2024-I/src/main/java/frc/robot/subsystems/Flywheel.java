package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;

public class Flywheel extends SubsystemBase{
    
    public static Flywheel flywheel;
    private Kraken flywheelBlack, flywheelOrange;
    
    public Flywheel(){
        flywheelBlack = new Kraken(Constants.FlywheelConstants.flywheel_black, Constants.GlobalConstants.canivore_name);
        flywheelOrange = new Kraken(Constants.FlywheelConstants.flywheel_orange, Constants.GlobalConstants.canivore_name);

        flywheelBlack.setSupplyCurrentLimit(Constants.FlywheelConstants.flywheel_current_limit);
        flywheelOrange.setSupplyCurrentLimit(Constants.FlywheelConstants.flywheel_current_limit);
        flywheelBlack.setForwardTorqueCurrentLimit(Constants.FlywheelConstants.flywheel_torque_limit);
        flywheelOrange.setForwardTorqueCurrentLimit(Constants.FlywheelConstants.flywheel_torque_limit);
        flywheelBlack.setReverseTorqueCurrentLimit(Constants.FlywheelConstants.flywheel_torque_limit);
        flywheelOrange.setReverseTorqueCurrentLimit(Constants.FlywheelConstants.flywheel_torque_limit);

        flywheelBlack.setInverted(false);
        flywheelOrange.setInverted(true);

        flywheelBlack.setVelocityPIDValues(0, 0, 0, 1, 0, 0, 0);
        flywheelOrange.setVelocityPIDValues(0, 0, 0, 1, 0, 0, 0);

        SmartDashboard.putNumber("flywheel P", 1);
        SmartDashboard.putNumber("flywheel kI", 0);
        SmartDashboard.putNumber("flywheel kS", 0);
        SmartDashboard.putNumber("black flywheel setpoint", 0);
        SmartDashboard.putNumber("orange flywheel setpoint", 0);
        SmartDashboard.putBoolean("Update PID values", false);
    }
    

    public void runOrangeFlywheelVelocitySetpoint(double speed){
        flywheelOrange.setVelocityTorqueFOC(speed/60);
        SmartDashboard.putNumber("orange flywheel setpoint", speed);
    }

    public void runBlackFlywheelVelocitySetpoint(double speed){
        flywheelBlack.setVelocityTorqueFOC(speed);
        SmartDashboard.putNumber("black flywheel setpoint", speed);
    }

    public static Flywheel getInstance(){
        if(flywheel == null){
            flywheel = new Flywheel();
        }
        return flywheel;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      // intakeMotor.set(TalonSRXControlMode.PercentOutput,SmartDashboard.getNumber("Intake
      // speed", 0));
      if(SmartDashboard.getBoolean("Update PID values", false)){
        flywheelBlack.setVelocityPIDValues(SmartDashboard.getNumber("flywheel kS", 0), 0, 0, SmartDashboard.getNumber("flywheel P", 1), SmartDashboard.getNumber("flywheel kI", 0), 0, 0);
        flywheelOrange.setVelocityPIDValues(SmartDashboard.getNumber("flywheel kS", 0), 0, 0, SmartDashboard.getNumber("flywheel P", 1), SmartDashboard.getNumber("flywheel kI", 0), 0, 0);
        SmartDashboard.putBoolean("Update PID values", false);
      }
        
        runOrangeFlywheelVelocitySetpoint(SmartDashboard.getNumber("orange flywheel setpoint", 0));
        runBlackFlywheelVelocitySetpoint(SmartDashboard.getNumber("black flywheel setpoint", 0));
    }

    
}
