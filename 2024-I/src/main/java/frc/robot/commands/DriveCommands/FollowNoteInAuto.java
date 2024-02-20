package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightIntake;
import frc.robot.utils.DriverOI;

public class FollowNoteInAuto extends Command {
    private Drivetrain drivetrain;
    private LimelightIntake limelightIntake;
    private double angleThreshold;
    private double targetAngle;

    private double FF;
    private double llTurn;
    private PIDController thetaController;

    private double currentAngle;
    private double error;

    private static final double EARLY_END_MIN_DURATION = 0.10;
    private static final double EARLY_END_MAX_DURATION = 0.25;
    private static final double EARLY_END_NO_NOTE_PCT = 0.80;
    private static final double NOT_SAME_NOTE_THRESHOLD = 2.5;
    private static final double SPEED = 1.5;
    
    private double startTime, timeLimit;
    private int totalFrameCount, doNotSeeFrameCount;
    private boolean endBecauseNoNote;
    private boolean hasGamePiece;

    private double lastTx;

    // Full constructor with all 6 parameters for the climb charge station
    // algorithm.
    public FollowNoteInAuto(double timeLimit) {
        drivetrain = Drivetrain.getInstance();
        limelightIntake = LimelightIntake.getInstance();
        
        this.timeLimit = timeLimit;

        angleThreshold = 0.5;

        error = 0.0;
        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        FF = 0.1;
        targetAngle = 0;
        llTurn = 0;

        SmartDashboard.putBoolean("has game piece", false);
        SmartDashboard.putNumber("DATA: Current TX", 0);
        SmartDashboard.putNumber("DATA: Last TX", 0);
        SmartDashboard.putNumber("DATA: Current-Last Difference", 0);

        doNotSeeFrameCount = 0;
        endBecauseNoNote = false;
        hasGamePiece = false;
        
        addRequirements(drivetrain);
   }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("has game piece", false);
        doNotSeeFrameCount = 0;
        totalFrameCount = 0;
        endBecauseNoNote = false;
        lastTx = Integer.MAX_VALUE;
        startTime = Timer.getFPGATimestamp();
        limelightIntake.setPipeline(1); 
    }

    @Override
    public void execute() {
        // double throttle = oi.getSwerveTranslation().getX();
        Translation2d position = new Translation2d(SPEED, 0.0);

        hasGamePiece = SmartDashboard.getBoolean("has game piece", false);

        llTurn = 0;

        boolean hasTarget = limelightIntake.hasTarget();

        // when the limelight has a target
        if (hasTarget) {
            // defined as number of consecutive cycles without note (see below)
            doNotSeeFrameCount = 0;
            // current angle
            currentAngle = limelightIntake.getTxAverage();
            // the next problem: if we stop seeing a note, but there is a note directly behind it
            // the robot will turn towards that note and everything breaks
            // so we say do not turn if the tx suddenly jumps
            // lastTx is initialized to Integer.MAX_VALUE, so we detect that here (makes the first game piece seen not seen as an extra note so the command won't end)
            // and only do turning if the absolute difference between the last angle and current angle is below the threshold
            if (lastTx == Integer.MAX_VALUE || Math.abs(lastTx - currentAngle) < NOT_SAME_NOTE_THRESHOLD) {
                lastTx = currentAngle;
                error = currentAngle - targetAngle;
                if (error < -angleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) + FF;
                else if (error > angleThreshold)
                    llTurn = thetaController.calculate(currentAngle, targetAngle) - FF;
            }
        }
        
        double elapsed = Timer.getFPGATimestamp() - startTime;
        if (elapsed <= EARLY_END_MAX_DURATION) {
            if (!hasTarget)
                doNotSeeFrameCount++;
            totalFrameCount++;
            if (elapsed >= EARLY_END_MIN_DURATION && doNotSeeFrameCount / totalFrameCount >= EARLY_END_NO_NOTE_PCT) {
                endBecauseNoNote = true;
                return;
            }
        }

        SmartDashboard.putNumber("follow note total frame count", totalFrameCount);
        SmartDashboard.putNumber("follow note no note frame count", doNotSeeFrameCount);
        SmartDashboard.putNumber("follow note percentage", doNotSeeFrameCount / totalFrameCount);

        drivetrain.drive(position, llTurn, false, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        if (endBecauseNoNote) {
            drivetrain.setIsParkedAuto(true);
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // endNow is a condition we invented earlier, hasGamePiece is whether the intake detects a piece
        // and set a cap on total time so that we do not get stuck in this command
        return endBecauseNoNote ||
            Timer.getFPGATimestamp() - startTime >= timeLimit ||
            hasGamePiece; // OR has the game piece
    }
}

