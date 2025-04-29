package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class Rate {
    private double lastMeasurement,lastVel,lastAccel,lastJerk,lastTimeStamp;

    public Rate(double d){
        lastMeasurement=d;
        lastVel=0;
        lastAccel=0;
        lastJerk=0;
        lastTimeStamp=Timer.getFPGATimestamp();
    }

    public void update(double d){
        double delta=Timer.getFPGATimestamp()-lastTimeStamp;
        lastTimeStamp=Timer.getFPGATimestamp();
        double currentVel=(d-lastMeasurement)/delta;
        double currentAccel=(currentVel-lastVel)/delta;
        double currentJerk=(currentAccel-lastAccel)/delta;

        lastMeasurement=d;
        lastVel=currentVel;
        lastAccel=currentAccel;
        lastJerk=currentJerk;
    }

    public double getVel() {return lastVel;}
    public double getAccel() {return lastAccel;}
    public double getJerk() {return lastJerk;}
}
