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
import frc.robot.utils.Constants;

// go to the closest shooting position
public class PathPlannerToShoot extends Command {
    private Drivetrain drivetrain;
    private Command followPathCommand;

    private double timeLimit;
    private double startTime;

    public PathPlannerToShoot(double timeLimit) {
        drivetrain = Drivetrain.getInstance();
        
        this.timeLimit = timeLimit;
        startTime = Timer.getFPGATimestamp();

        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        // if we are parked (went to midline and no note) then do not do anything
        if (drivetrain.getIsParkedAuto()) {
            followPathCommand = null;
            return;
        }

        // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html

        // find the closest shooting location to go to using distance formula
        var currentOdometry = drivetrain.getPose();
        double currentX = currentOdometry.getX();
        double currentY = currentOdometry.getY();

        double minDistance = Integer.MAX_VALUE;
        double[] point = Constants.AutoConstants.shootingPositions[0];

        for (double[] shootingPoint : Constants.AutoConstants.shootingPositions) {
            // distance formula
            double distance = Math.sqrt(Math.pow(shootingPoint[0] - currentX, 2) + Math.pow(shootingPoint[1] - currentY, 2));
            // compare against current minimum
            if (distance < minDistance) {
                minDistance = distance;
                point = shootingPoint;
            }
        }

        // pathfindToPose doesn't flip coordinates for red side. Have to do that manually
        double xTarget, yTarget, turnTarget;
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        // point[0] = x, point[1] = y, point[2] = rotation
        xTarget = isRed ? 16.542 - point[0] : point[0];
        yTarget = point[1];
        turnTarget = isRed ? point[2] - 180 : point[2];

        // pathplanner on the fly pathfinding
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