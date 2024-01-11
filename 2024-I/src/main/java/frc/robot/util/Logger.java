package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class Logger {
    private static Logger instance;
    private BooleanLogEntry booleanLog;
    private DoubleLogEntry doubleLog;
    private StringLogEntry stringLog;
    private DoubleArrayLogEntry doubleArrayLog;
    private DataLog log = DataLogManager.getLog();
    private double lastTeleopEnable;


    public static Logger getInstance(){
        if (instance == null){
            instance = new Logger();
        }
        return instance;
    }

    public Logger(){
        booleanLog = new BooleanLogEntry(log, "/Data/BooleanLog");
        doubleLog = new DoubleLogEntry(log,"/Data/DoubleLog");
        stringLog = new StringLogEntry(log,"/Data/StringLog");
        doubleArrayLog = new DoubleArrayLogEntry(log,"/Data/DoubleArrayLog");

    }
    public void LogEvent(String event, Boolean isStart){
        stringLog.append(event + (isStart? " Started": " Ended"));

    }

    public void UpdateLogs(){

    }

    public void SignalRobotEnable(){
        lastTeleopEnable = Timer.getFPGATimestamp();
    }
}
