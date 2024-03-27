package frc.robot.commands.AutoCommands;

import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.FollowNoteInAuto;
import frc.robot.commands.DriveCommands.TargetInAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;

public class CustomizableMidlineAuto extends Command {
    enum CMAState {
        DRIVING_TO_NOTE,
        SEEKING_NOTE,
        DRIVING_TO_SHOOT,
        SHOOTING
    }

    private Superstructure superstructure;
    private Drivetrain drivetrain;
    private LimelightIntake limelightIntake;
    private Hopper hopper;
    
    private Command pathFollowingCommand, noteSeekingCommand, targetingAndScoringCommand;

    private int[] targets;
    private int currentNote;

    CMAState state;

    public CustomizableMidlineAuto(int[] notes) {
        superstructure = Superstructure.getInstance();
        drivetrain = Drivetrain.getInstance();
        limelightIntake = LimelightIntake.getInstance();
        hopper = Hopper.getInstance();

        targets = notes;
        currentNote = -1;

        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        currentNote = -1;
        pathFindToNote();
        noteSeekingCommand = null;
        targetingAndScoringCommand = null;
    }

    @Override
    public void execute() {
        switch (state) {
            case DRIVING_TO_NOTE:
                // found a note: stop following the path, go and note seek
                // make the note seeking command, initialize it and execute it
                // by switching to the note seeking state
                if (limelightIntake.hasTarget()) {
                    noteSeekingCommand = new FollowNoteInAuto(1);
                    noteSeekingCommand.initialize();
                    state = CMAState.SEEKING_NOTE;
                }
                if (pathFollowingCommand != null) {
                    // if there actually is a note there, this code should not execute
                    // we should have switched to the seeking note state
                    // so if the command finishes (ie drove to the right place), get the next note
                    if (pathFollowingCommand.isFinished())
                        pathFindToNote();
                    else
                        pathFollowingCommand.execute();
                }
                break;
            case DRIVING_TO_SHOOT:
                if (pathFollowingCommand != null) {
                    if (pathFollowingCommand.isFinished()) {
                        targetingAndScoringCommand = new SequentialCommandGroup(
                            new TargetInAuto(),
                            new Score()
                        );
                        targetingAndScoringCommand.initialize();
                        state = CMAState.SHOOTING;
                    }
                    else
                        pathFollowingCommand.execute();
                }
                break;
            case SEEKING_NOTE:
                // run the note seeking command
                if (noteSeekingCommand != null) {
                    // if the command is done
                    if (noteSeekingCommand.isFinished()) {
                        // has a game piece: happy. path find, prepare to shoot and switch state
                        if (hopper.hasGamepiece()) {
                            pathFindToShoot();
                            superstructure.requestState(SuperstructureState.LL_PREP);
                        }
                        // no game piece: sad. path find to the next note and switch state
                        else
                            pathFindToNote();
                    }
                    // if not finished then just execute
                    else
                        noteSeekingCommand.execute();
                    // IMPORTANT: do not run the end function. do not mess with isParkedAuto (should delete tbh)
                }
                break;
            case SHOOTING:
                if (targetingAndScoringCommand != null) {
                    if (targetingAndScoringCommand.isFinished()) {
                        targetingAndScoringCommand.end(false);
                        pathFindToShoot();
                    }
                    else
                        targetingAndScoringCommand.execute();;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathFollowingCommand != null)
            pathFollowingCommand.end(interrupted);
        if (noteSeekingCommand != null)
            noteSeekingCommand.end(interrupted);
        if (targetingAndScoringCommand != null)
            targetingAndScoringCommand.end(interrupted);
    }

    @Override
    // honestly if you're going to run this command you will probably run it for the whole auto period
    public boolean isFinished() {
        return false;
    }    

    private void pathFindToNote() {
        currentNote++;

        // did all the notes, time to end
        if (currentNote >= targets.length) {
            pathFollowingCommand = null;
            return;
        }

        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        double targetX = AutoConstants.kNotePositions[targets[currentNote]][0];
        double targetY = AutoConstants.kNotePositions[targets[currentNote]][1];
        double turnTarget = isRed ? 0 : 180;

        if (isRed)
            targetX = 16.542 - targetX;

        // PathPlanner on the fly pathfinding code
        PathConstraints constraints = new PathConstraints(
            3.5, 3.0, Units.degreesToRadians(520), Units.degreesToRadians(540)
        );

        Pose2d targetPose = new Pose2d(targetX, targetY, Rotation2d.fromDegrees(turnTarget));
        pathFollowingCommand = AutoBuilder.pathfindToPose(
            targetPose, constraints, AutoConstants.kFollowNoteSpeed, 0.0
        );

        pathFollowingCommand.initialize();
        state = CMAState.DRIVING_TO_NOTE;

        superstructure.requestState(SuperstructureState.GROUND_INTAKE);

        // make the robot turn towards the speaker using the odometry
        // too many holonomic rotations might break the odoemtry though
        // but that is what we have note seeking and botpose for!
        PPHolonomicDriveController.setRotationTargetOverride(this::turnToNoteSupplier);
    }

    public void pathFindToShoot() {
        // find the closest shooting location to go to using distance formula
        var currentOdometry = drivetrain.getPose();
        double currentX = currentOdometry.getX();
        double currentY = currentOdometry.getY();

        double minDistance = Integer.MAX_VALUE;
        double[] point = Constants.AutoConstants.kShootingPositions[0];

        for (double[] shootingPoint : Constants.AutoConstants.kShootingPositions) {
            // distance formula
            double distance = Math.sqrt(Math.pow(shootingPoint[0] - currentX, 2) + Math.pow(shootingPoint[1] - currentY, 2));
            // compare against current minimum
            if (distance < minDistance) {
                minDistance = distance;
                point = shootingPoint;
            }
        }

        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        double xTarget = isRed ? 16.542 - point[0] : point[0];
        double yTarget = point[1];
        double turnTarget = isRed ? point[2] - 90 : point[2];

        // PathPlanner on the fly pathfinding code
        PathConstraints constraints = new PathConstraints(
            3.5, 3.0, Units.degreesToRadians(520), Units.degreesToRadians(540)
        );

        Pose2d targetPose = new Pose2d(xTarget, yTarget, Rotation2d.fromDegrees(turnTarget));
        pathFollowingCommand = AutoBuilder.pathfindToPose(
            targetPose, constraints, AutoConstants.kFollowNoteSpeed, 0.0
        );

        pathFollowingCommand.initialize();
        state = CMAState.DRIVING_TO_SHOOT;

        // don't want to override anything, supply it with nothing to do this
        PPHolonomicDriveController.setRotationTargetOverride(this::nothingSupplier);
    }

    private Optional<Rotation2d> turnToNoteSupplier() {
        // generally face the note so the note seek can actually work
        var currentOdometry = drivetrain.getPose();
        double currentX = currentOdometry.getX();
        double currentY = currentOdometry.getY();
        double currentAngle = currentOdometry.getRotation().getDegrees();

        // position of the current note
        double targetX = AutoConstants.kNotePositions[targets[currentNote]][0];
        double targetY = AutoConstants.kNotePositions[targets[currentNote]][1];
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            targetX = 16.542 - targetX;

        // calculate the angle the robot needs to be in relative to the note
        // (add 180 because the back needs to be turned to the note)
        // then calculate the necessary turn
        double desiredAngle = Math.toDegrees(Math.atan2(targetY - currentY, targetX - currentX)) + 180;
        double turnAngle = desiredAngle - currentAngle;

        return Optional.of(Rotation2d.fromDegrees(turnAngle));
    }

    private Optional<Rotation2d> nothingSupplier() {
        return Optional.empty();
    }
}