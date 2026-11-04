package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Pedro autonomous version of DriveToTargetCommand: uses the webcam to line up with the
 * target AprilTag cluster (right distance, centered, square to it) before shooting.
 * It drives through Pedro, so Pedro keeps tracking the position while aiming.
 * When it finishes, the robot holds that spot while it shoots.
 *
 * Tuning numbers are the same ones the old autonomous uses, in Constants.AutoConstants:
 * DESIRED_DISTANCE, SPEED_GAIN, STRAFE_GAIN, TURN_GAIN, MAX_AUTO_*, RANGE_TOLERANCE, ANGLE_TOLERANCE.
 *
 * Note: this season's tags move, so the camera is only used for aiming, not for fixing the
 * robot's field position (the FTC SDK notes say BIOBUZZ tags can't be used for that).
 */
public class PedroAimCommand extends CommandBase {

    private final PedroDriveSubsystem drive;
    private final VisionSubsystem vision;

    public PedroAimCommand(PedroDriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    private boolean holding = false;

    @Override
    public void initialize() {
        drive.holdHere();
        holding = true;
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // Lost the target: stay put (once, at the spot it lost it) and wait for the camera to find it again.
            if (!holding) {
                drive.holdHere();
                holding = true;
            }
            return;
        }
        holding = false;

        double rangeError   = vision.getTargetRange() - AutoConstants.DESIRED_DISTANCE;
        double headingError = vision.getTargetBearing();  // positive = target is to the left
        double yawError     = vision.getTargetYaw();

        // Pedro's directions: positive strafe = LEFT, positive turn = LEFT (counterclockwise).
        double forward = clip(rangeError * AutoConstants.SPEED_GAIN, AutoConstants.MAX_AUTO_SPEED);
        double strafe  = clip(-yawError * AutoConstants.STRAFE_GAIN, AutoConstants.MAX_AUTO_STRAFE);
        double turn    = clip(headingError * AutoConstants.TURN_GAIN, AutoConstants.MAX_AUTO_TURN);

        drive.getFollower().manual(forward, strafe, turn);
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget()
                && Math.abs(vision.getTargetRange() - AutoConstants.DESIRED_DISTANCE) < AutoConstants.RANGE_TOLERANCE
                && Math.abs(vision.getTargetBearing()) < AutoConstants.ANGLE_TOLERANCE
                && Math.abs(vision.getTargetYaw()) < AutoConstants.ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drive.holdHere();  // keep this aimed spot while shooting
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
