package frc.robot.Shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab tab;

    public abstract void createEntries();

    protected GenericEntry createNumberEntry(String name) {
        return tab.add(name, 0.0).withSize(2, 1).getEntry();
    }

    protected GenericEntry createStringEntry(String name) {
        return tab.add(name, "").withSize(2, 1).getEntry();
    }

    public abstract void update();

    /* Truncates number to 2 decimal places for cleaner numbers */
    protected double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }

    public ShuffleboardTab getTab() {
        return tab;
    }

}