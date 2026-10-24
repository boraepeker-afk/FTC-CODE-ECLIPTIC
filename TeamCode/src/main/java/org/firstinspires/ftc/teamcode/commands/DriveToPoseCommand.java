package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.PathConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*
 * Drives to a spot on the field, measured from where autonomous started, and turns to a heading
 * at the same time. It uses the drivetrain's encoder position tracking.
 *
 *   x       = inches forward from the start
 *   y       = inches to the LEFT of the start (negative = right)
 *   heading = degrees to face, left turn positive (0 = the way the robot faced at the start)
 *
 * It slows down as it gets close, and finishes when it's within the tolerances in
 * Constants.PathConstants. Always give it a timeout (withTimeout) so auto can't get stuck.
 */
public class DriveToPoseCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final double targetX, targetY, targetHeadingDeg, maxSpeed;

    public DriveToPoseCommand(DriveSubsystem drive, double x, double y, double headingDeg, double maxSpeed) {
        this.drive = drive;
        this.targetX = x;
        this.targetY = y;
        this.targetHeadingDeg = headingDeg;
        this.maxSpeed = maxSpeed;
        addRequirements(drive);
    }

    /** Same, using a spot written as {x, y, heading}. */
    public DriveToPoseCommand(DriveSubsystem drive, double[] pose, double maxSpeed) {
        this(drive, pose[0], pose[1], pose[2], maxSpeed);
    }

    @Override
    public void execute() {
        double errorX = targetX - drive.getX();
        double errorY = targetY - drive.getY();
        double headingError = Math.toDegrees(DriveSubsystem.angleWrap(
                Math.toRadians(targetHeadingDeg - drive.getPoseHeadingDegrees())));

        // Turn the field error into the robot's own forward / sideways directions.
        double h = Math.toRadians(drive.getPoseHeadingDegrees());
        double forwardError = errorX * Math.cos(h) + errorY * Math.sin(h);
        double leftError    = -errorX * Math.sin(h) + errorY * Math.cos(h);

        // Power grows with distance ("P control"), plus a small push so it doesn't stall just short.
        double forward = boost(forwardError * PathConstants.DRIVE_GAIN);
        double strafe  = boost(-leftError * PathConstants.DRIVE_GAIN);   // drive(): positive strafe = right
        double turn    = clip(-headingError * PathConstants.TURN_GAIN, PathConstants.MAX_TURN);  // positive turn = right

        // Keep the overall speed under maxSpeed while keeping the direction.
        double biggest = Math.max(Math.abs(forward), Math.abs(strafe));
        if (biggest > maxSpeed) {
            forward = forward / biggest * maxSpeed;
            strafe = strafe / biggest * maxSpeed;
        }
        drive.drive(forward, strafe, turn, 1.0);
    }

    @Override
    public boolean isFinished() {
        double distance = Math.hypot(targetX - drive.getX(), targetY - drive.getY());
        double headingError = Math.abs(Math.toDegrees(DriveSubsystem.angleWrap(
                Math.toRadians(targetHeadingDeg - drive.getPoseHeadingDegrees()))));
        return distance < PathConstants.POSITION_TOLERANCE && headingError < PathConstants.HEADING_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private static double boost(double power) {
        if (Math.abs(power) < 1e-6) return 0;
        return power + Math.signum(power) * PathConstants.MIN_POWER;
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
