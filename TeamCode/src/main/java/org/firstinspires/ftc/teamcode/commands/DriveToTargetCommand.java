package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Drives the robot up to the AprilTag target the camera sees and lines up squarely in front of it.
 * It does three things at once, like the official FTC sample "RobotAutoDriveToAprilTagOmni":
 *   - drives forward/back until the camera is DESIRED_DISTANCE inches from the target
 *   - turns to keep the target centered in the camera
 *   - strafes sideways to get directly in front of the target
 *
 * This season (BIOBUZZ) the target for a tag cluster is the center of the Cell opening.
 *
 * Finishes when the robot is lined up within the tolerances below.
 * If the camera loses the target, the robot stops and waits for it to reappear.
 * Give it a timeout (withTimeout) so autonomous can't get stuck here.
 */
public class DriveToTargetCommand extends CommandBase {

    // The tuning numbers are in Constants.AutoConstants.
    public static final double DESIRED_DISTANCE = Constants.AutoConstants.DESIRED_DISTANCE;
    private static final double SPEED_GAIN      = Constants.AutoConstants.SPEED_GAIN;
    private static final double STRAFE_GAIN     = Constants.AutoConstants.STRAFE_GAIN;
    private static final double TURN_GAIN       = Constants.AutoConstants.TURN_GAIN;
    private static final double MAX_AUTO_SPEED  = Constants.AutoConstants.MAX_AUTO_SPEED;
    private static final double MAX_AUTO_STRAFE = Constants.AutoConstants.MAX_AUTO_STRAFE;
    private static final double MAX_AUTO_TURN   = Constants.AutoConstants.MAX_AUTO_TURN;
    private static final double RANGE_TOLERANCE = Constants.AutoConstants.RANGE_TOLERANCE;
    private static final double ANGLE_TOLERANCE = Constants.AutoConstants.ANGLE_TOLERANCE;

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    public DriveToTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            drive.stop();  // lost the target: hold still and wait for the camera to find it again
            return;
        }

        double rangeError   = vision.getTargetRange() - DESIRED_DISTANCE;
        double headingError = vision.getTargetBearing();  // positive = target is to the left
        double yawError     = vision.getTargetYaw();

        double forward = clip(rangeError * SPEED_GAIN, MAX_AUTO_SPEED);
        // In drive(), positive strafe is to the right and positive turn is to the right,
        // so the signs here are flipped compared to the official sample (where left is positive).
        double strafe  = clip(yawError * STRAFE_GAIN, MAX_AUTO_STRAFE);
        double turn    = clip(-headingError * TURN_GAIN, MAX_AUTO_TURN);

        drive.drive(forward, strafe, turn, 1.0);
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget()
                && Math.abs(vision.getTargetRange() - DESIRED_DISTANCE) < RANGE_TOLERANCE
                && Math.abs(vision.getTargetBearing()) < ANGLE_TOLERANCE
                && Math.abs(vision.getTargetYaw()) < ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
