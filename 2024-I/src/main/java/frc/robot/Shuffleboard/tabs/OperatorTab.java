package frc.robot.Shuffleboard.tabs;

import java.util.Enumeration;
import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shuffleboard.ShuffleboardTabBase;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.FlywheelConstants;
import frc.robot.utils.Constants.LimelightConstants;

public class OperatorTab extends ShuffleboardTabBase {
        private SendableChooser<Command> autoChooser;
        private SendableChooser<String> autoSetupChooser;

        private ComplexWidget autoChooserWidget, autoSetupWidget, intakeCameraWidget, shooterCameraWidget;

        private Arm arm;
        private Autonomous autonomous;
        private Flywheel flywheel;
        private Hopper hopper;
        private Intake intake;
        private Superstructure superstructure;
        private LimelightShooter limelightShooter;

        private GenericEntry stateEntry, armAngleEntry,
                        flywheelAtRPMEntry, redTargetingOffsetEntry, llDistanceMultiplierEntry, llDistanceEntry,
                        ampAngleEntry,
                        blueTargetingOffsetEntry, isIndexedOverrideEntry, topSensorEntry, bottomSensorEntry,
                        hasGamePieceEntry, useOdometryTargetEntry, lobPassAngleEntry, lobPassFlywheelMultiplierEntry;

        public OperatorTab() {
                arm = Arm.getInstance();
                autonomous = Autonomous.getInstance();
                flywheel = Flywheel.getInstance();
                hopper = Hopper.getInstance();
                intake = Intake.getInstance();
                superstructure = Superstructure.getInstance();
                limelightShooter = LimelightShooter.getInstance();
        }

        public void createEntries() {
                tab = Shuffleboard.getTab("Operator");

                try {
                        stateEntry = tab.add("State", "STOW")
                                        .withSize(2, 1)
                                        .withPosition(14, 0)
                                        .getEntry();

                        armAngleEntry = tab.add("Arm Angle", 0.0)
                                        .withSize(1, 1)
                                        .withPosition(13, 1)
                                        .getEntry();

                        useOdometryTargetEntry = tab.add("Odo Target", true)
                                        .withSize(1, 1)
                                        .withPosition(12, 1)
                                        .getEntry();

                        intakeCameraWidget = tab.addCamera("Intake Camera", "LL Intake", "http://10.58.95.53:5800")
                                        .withSize(5, 4)
                                        .withPosition(5, 0);

                        shooterCameraWidget = tab.addCamera("Shooter Camera", "LL Shooter", "http://10.58.95.41:5800")
                                        .withSize(5, 4)
                                        .withPosition(0, 0);

                        llDistanceEntry = tab.add("LL Distance", 0.0)
                                        .withSize(1, 1)
                                        .withPosition(14, 1)
                                        .getEntry();

                        llDistanceMultiplierEntry = tab.add("LL Dist Multiplier", ArmConstants.kArmLLDistMultiplier)
                                        .withSize(1, 1)
                                        .withPosition(13, 2)
                                        .getEntry();

                        ampAngleEntry = tab.add("Amp Angle", ArmConstants.kArmAmpPosition)
                                        .withSize(1, 1)
                                        .withPosition(14, 2)
                                        .getEntry();

                        redTargetingOffsetEntry = tab.add("Red Target", LimelightConstants.kRedTargetTarget)
                                        .withSize(1, 1)
                                        .withPosition(12, 0)
                                        .getEntry();

                        blueTargetingOffsetEntry = tab.add("Blue Target", LimelightConstants.kBlueTargetTarget)
                                        .withSize(1, 1)
                                        .withPosition(13, 0)
                                        .getEntry();

                        flywheelAtRPMEntry = tab.add("Flywheel At RPM?", false)
                                        .withSize(1, 1)
                                        .withPosition(15, 1)
                                        .getEntry();

                        // Return to this later
                        // isIndexedOverrideEntry = tab.add("Piece Indexed Override", false)
                        // .withWidget(BuiltInWidgets.kToggleButton)
                        // .withSize(2,1)
                        // .withPosition(3, 0)
                        // .getEntry();

                        topSensorEntry = tab.add("Top Sensor?", false)
                                        .withSize(1, 1)
                                        .withPosition(11, 0)
                                        .getEntry();

                        bottomSensorEntry = tab.add("Bottom Sensor?", false)
                                        .withSize(1, 1)
                                        .withPosition(11, 1)
                                        .getEntry();

                        hasGamePieceEntry = tab.add("Has Gamepiece?", false)
                                        .withSize(5, 5)
                                        .withPosition(16, 0)
                                        .getEntry();

                        lobPassAngleEntry = tab.add("Pass Angle", ArmConstants.kArmLobPassPosition)
                                        .withSize(1,1)
                                        .withPosition(12,2) //todo update
                                        .getEntry();

                        lobPassFlywheelMultiplierEntry = tab.add("Pass Multi", FlywheelConstants.kFlywheelLobPassSpeedMultiplier)
                                        .withSize(1,1)
                                        .withPosition(11,2)
                                        .getEntry();
                } catch (IllegalArgumentException e) {
                }
        }

        @Override
        public void update() {
                try {
                        armAngleEntry.setDouble(arm.getArmAngleDegrees());
                        llDistanceEntry.setDouble(limelightShooter.getDistance());
                        useOdometryTargetEntry.setBoolean(DriverOI.getInstance().isUsingOdometryTarget());

                        flywheelAtRPMEntry.getBoolean(flywheel.isAtRPM());
                        
                        limelightShooter.setRedTargetingOffset(redTargetingOffsetEntry.getDouble(LimelightConstants.kRedTargetTarget));
                        limelightShooter.setBlueTargetingOffset(blueTargetingOffsetEntry.getDouble(LimelightConstants.kBlueTargetTarget));

                        flywheel.setFlywheelLobPassMultiplier(lobPassFlywheelMultiplierEntry.getDouble(FlywheelConstants.kFlywheelLobPassSpeedMultiplier));

                        arm.setLLDistanceMultiplier(llDistanceMultiplierEntry.getDouble(ArmConstants.kArmLLDistMultiplier));
                        arm.setAmpScoringAngle(ampAngleEntry.getDouble(ArmConstants.kArmAmpPosition));
                        arm.setLobPassAngle(lobPassAngleEntry.getDouble(ArmConstants.kArmLobPassPosition));

                        // isIndexedOverrideEntry.getBoolean(false); // Return to this later
                        stateEntry.setString(superstructure.getRobotState());

                        topSensorEntry.setBoolean(hopper.getTopSensor());
                        bottomSensorEntry.setBoolean(hopper.getBottomSensor());

                        hasGamePieceEntry.setBoolean(hopper.hasGamepiece());
                } catch (IllegalArgumentException e) {
                }
        }

        public void configureAutoSelector() {
                autoChooser = autonomous.getAutoChooser();
                autoChooserWidget = tab.add("Auto routine", autoChooser)
                                .withSize(2, 1)
                                .withPosition(11, 3);
        }

        public void configureAutoSetupSelector() {
                autoSetupChooser = new SendableChooser<String>();
                autoSetupChooser.setDefaultOption("NONE/TELEOP", "NONE/TELEOP");
                autoSetupChooser.addOption("SOURCE", "SOURCE");
                autoSetupChooser.addOption("AMP", "AMP");
                autoSetupChooser.addOption("CENTER", "CENTER");
                autoSetupWidget = tab.add("Auto setup", autoSetupChooser)
                                .withSize(2, 1)
                                .withPosition(13, 3);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public double getGyroOffsetForTeleop() { // only for red side. Negated for blue side in main.
                if (autoSetupChooser.getSelected().equals("NONE/TELEOP")) {
                        return 0;
                } else if (autoSetupChooser.getSelected().equals("SOURCE")) {
                        return -120.0;
                } else if (autoSetupChooser.getSelected().equals("AMP")) {
                        return 120.0;
                } else if (autoSetupChooser.getSelected().equals("CENTER")) {
                        return 180.0;
                } else {
                        return 0.0;
                }
        }

}
