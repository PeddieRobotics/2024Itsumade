
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.OI;

public class RobotContainer {
  
  private final Intake intake;
  private final Flywheel flywheel;
  private final OI oi;
  //private final Cartridge cartridge;
 
  public RobotContainer() {

 
    intake = Intake.getInstance();
    flywheel = Flywheel.getInstance();
    oi = OI.getInstance();
    //cartridge = Cartridge.getInstance();

  }

 
}
