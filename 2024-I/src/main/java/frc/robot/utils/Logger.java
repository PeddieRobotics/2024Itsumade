package frc.robot.utils;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Flywheel;

public class Logger {
    private static Logger logger;
    private DoubleLogEntry flywheelOrangeCurrentEntry;
    private DataLog log = DataLogManager.getLog();
    private Flywheel flywheel;

    public Logger(){
        flywheel = Flywheel.getInstance();
        flywheelOrangeCurrentEntry = new DoubleLogEntry(log, "/Flywheel/Orange Current");
    }

    public void updateLogs(){
        flywheelOrangeCurrentEntry.append(flywheel.getOrangeSupplyCurrent());
    }

    public static Logger getInstance(){
        if(logger == null){
            logger = new Logger();
        }
        return logger;
    }
    
}

