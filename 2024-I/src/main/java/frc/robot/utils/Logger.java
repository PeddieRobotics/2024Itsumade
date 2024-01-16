package frc.robot.utils;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {
    private static Logger logger;
    private BooleanLogEntry booleanLog;
    private StringLogEntry stringLog;
    private DoubleLogEntry doubleLog;
    private DataLog dataLog = DataLogManager.getLog();

    public Logger(){
        booleanLog = new BooleanLogEntry(dataLog, "Data/BooleanLog");
        stringLog = new StringLogEntry(dataLog, "Data/StringLog");
        doubleLog = new DoubleLogEntry(dataLog, "Data/DoubleLog");
    }

    public void logEvent(String event, boolean started){
        if(started){
            stringLog.append(event + " started");   
        }else stringLog.append(event + " ended");

        stringLog.append(event + (started ?" started": " ended"));
    }

    public void updateLog(){}

    public static Logger getInstance(){
        if(logger == null){
            logger = new Logger();
        }return logger;
    }
}
