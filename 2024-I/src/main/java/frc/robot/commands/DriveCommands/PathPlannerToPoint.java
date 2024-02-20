package frc.robot.commands.DriveCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// code from these people who figured this out
// https://github.com/Hamosad1657/MiniProject2023/blob/chassis/src/main/java/frc/robot/commands/drivetrain/FollowGeneratedTrajectoryCommand.java

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants;

public class PathPlannerToPoint extends Command {
    private Drivetrain drivetrain;
    private Command followPathCommand;

    private double xTargetInitial, turnTargetInitial;
    private double xTarget, yTarget, turnTarget;

    private double timeLimit;
    private double startTime;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public PathPlannerToPoint(double x, double y, double theta, double timeLimit) {
        drivetrain = Drivetrain.getInstance();
        
        xTargetInitial = x; 
        yTarget = y; 
        turnTargetInitial = theta;

        this.timeLimit = timeLimit;
        startTime = Timer.getFPGATimestamp();

        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        if (drivetrain.getIsParkedAuto()) {
            followPathCommand = null;
            return;
        }
        
        // pathfindToPose doesn't flip coordinates for red side. Have to do that manually
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        turnTarget = isRed ? turnTargetInitial - 180 : turnTargetInitial;
        xTarget = isRed ? 16.542 - xTargetInitial : xTargetInitial;

        // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html
        // var currentOdometry = drivetrain.getPose();

        // SmartDashboard.putNumber("otf target x", xTarget);
        // SmartDashboard.putNumber("otf target y", yTarget);
        // SmartDashboard.putNumber("otf target turn", turnTarget);

        // SmartDashboard.putNumber("otf current pose x", currentOdometry.getX());
        // SmartDashboard.putNumber("otf current pose y", currentOdometry.getY());
        // SmartDashboard.putNumber("otf current pose turn", currentOdometry.getRotation().getDegrees());

        PathConstraints constraints = new PathConstraints(
            1.5, 1.5, Units.degreesToRadians(720), Units.degreesToRadians(720)
        );

        Pose2d targetPose = new Pose2d(xTarget, yTarget, Rotation2d.fromDegrees(turnTarget));
        followPathCommand = AutoBuilder.pathfindToPose(
            targetPose, constraints, 0.0, 0.0
        );

        followPathCommand.initialize();
    }

    @Override
    public void execute() {
        if (followPathCommand != null)
            followPathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (followPathCommand != null)
            followPathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return followPathCommand == null || 
            followPathCommand.isFinished() ||
            Timer.getFPGATimestamp() - startTime >= timeLimit; 
    }
}