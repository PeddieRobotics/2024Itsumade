
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  
  private final Intake intake;
  private final Flywheel flywheel;
  //private final Cartridge cartridge;
 
  public RobotContainer() {

 
    intake = Intake.getInstance();
    flywheel = Flywheel.getInstance();
    //cartridge = Cartridge.getInstance();

  }

 
}
