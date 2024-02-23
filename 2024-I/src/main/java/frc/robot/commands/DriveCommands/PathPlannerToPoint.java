package frc.robot.commands.DriveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

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
        // if the robot is parked (we do not want it to move anymore) then do not do anything
        if (drivetrain.getIsParkedAuto()) {
            followPathCommand = null;
            return;
        }
        
        // pathfindToPose doesn't flip coordinates for red side. Have to do that manually
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        turnTarget = isRed ? turnTargetInitial - 180 : turnTargetInitial;
        xTarget = isRed ? 16.542 - xTargetInitial : xTargetInitial;

        // PathPlanner on the fly pathfinding code
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