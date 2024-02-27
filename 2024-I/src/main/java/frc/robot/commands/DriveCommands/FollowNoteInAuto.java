package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.LimelightConstants;

public class FollowNoteInAuto extends Command {
    private Drivetrain drivetrain;
    private LimelightIntake limelightIntake;
    private Hopper hopper;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;
    
    private double startTime, timeLimit;
    private int totalFrameCount, doNotSeeFrameCount;
    private boolean endBecauseNoNote;
    private double lastTx;

    public FollowNoteInAuto(double timeLimit) {
        drivetrain = Drivetrain.getInstance();
        limelightIntake = LimelightIntake.getInstance();
        hopper = Hopper.getInstance();
        
        this.timeLimit = timeLimit;

        error = 0.0;
        thetaController = new PIDController(LimelightConstants.kFollowNoteTurnP, LimelightConstants.kFollowNoteTurnI,
            LimelightConstants.kFollowNoteTurnD);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        llTurn = 0;

        doNotSeeFrameCount = 0;
        endBecauseNoNote = false;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        doNotSeeFrameCount = 0;
        totalFrameCount = 0;
        endBecauseNoNote = false;
        lastTx = Integer.MAX_VALUE;
        startTime = Timer.getFPGATimestamp();
        limelightIntake.setPipeline(LimelightConstants.kIntakeNotePipeline); 
    }

    @Override
    public void execute() {
        Translation2d position = new Translation2d(AutoConstants.kFollowNoteSpeed, 0.0);

        llTurn = 0;
        boolean hasTarget = limelightIntake.hasTarget();

        // when the limelight has a target
        if (hasTarget) {
            // defined as number of consecutive cycles without note (see below)
            doNotSeeFrameCount = 0;
            // current angle
            currentAngle = limelightIntake.getTxAverage();
            // the next problem: if we stop seeing a note, but there is a note directly
            // behind it
            // the robot will turn towards that note and everything breaks
            // so we say do not turn if the tx suddenly jumps
            // lastTx is initialized to Integer.MAX_VALUE, so we detect that here (makes the first game piece seen not seen as an extra note so the command won't end)
            // and only do turning if the absolute difference between the last angle and current angle is below the threshold
            if (lastTx == Integer.MAX_VALUE || Math.abs(lastTx - currentAngle) < AutoConstants.kFollowNoteNotSameNoteThresh) {
                lastTx = currentAngle;
                error = currentAngle - targetAngle;
                if (error < -LimelightConstants.kFollowNoteAngleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
                else if (error > LimelightConstants.kFollowNoteAngleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
            }
        }

        // if we don't see a note when starting the command, then do not follow anything
        // and park the robot at the current position
        // we end the command if three conditions are met:
        // 1. in the first EARLY_END_MAX_DURATION seconds
        // 2. the proportion number of frames where there is no note is above
        // EARLY_END_NO_NOTE_PCT
        // 3. higher than EARLY_END_MIN_DURATION has passed, so we need to not see a
        // note for a duration of time before we conclude there is no note
        double elapsed = Timer.getFPGATimestamp() - startTime;
        if (elapsed <= AutoConstants.kFollowNoteEarlyEndMaxDuration) {
            if (!hasTarget)
                doNotSeeFrameCount++;
            totalFrameCount++;
            if (elapsed >= AutoConstants.kFollowNoteEarlyEndMinDuration &&
                doNotSeeFrameCount / totalFrameCount >= AutoConstants.kFollowNoteNoNotePercent) {
                endBecauseNoNote = true;
                return;
            }
        }

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: ARE WE SURE THIS IS A GOOD IDEA???
        // park the robot at the current location
        // set isParkedAuto to true in drivetrain
        // this disables other PathPlanner OTF commands and PathPlanner paths
        if (endBecauseNoNote) {
            drivetrain.setIsParkedAuto(true);
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // endBecauseNoNote is a condition we invented earlier
        // and set a cap on total time so that we do not get stuck in this command
        return endBecauseNoNote ||
            Timer.getFPGATimestamp() - startTime >= timeLimit ||
            hopper.isGamepieceIndexed(); // OR has the game piece
    }
}
