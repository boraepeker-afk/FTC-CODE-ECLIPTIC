package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

import java.util.function.Supplier;

import static com.pedropathing.api.Paths.line;

/*
 * Drives one Pedro path. Finishes when the robot reaches the end of the path; after that
 * Pedro holds the robot at the end spot until the next drive command.
 * Add .withTimeout(ms) so a stuck robot can't use up all of autonomous.
 *
 * Two ways to use it:
 *   FollowPathCommand.goTo(drive, x, y, headingDegrees, maxSpeed)
 *       - a straight line from wherever the robot IS when the command starts, to that spot.
 *   new FollowPathCommand(drive, () -> someCurve)
 *       - any Pedro path you build yourself (see Pedro's docs for curve(...)).
 */
public class FollowPathCommand extends CommandBase {

    private final PedroDriveSubsystem drive;
    private final Supplier<Path> pathMaker;
    private boolean nothingToDo = false;

    /** The path is made when the command starts, so it can begin from the robot's real position. */
    public FollowPathCommand(PedroDriveSubsystem drive, Supplier<Path> pathMaker) {
        this.drive = drive;
        this.pathMaker = pathMaker;
        addRequirements(drive);
    }

    /**
     * Straight line from the robot's position at that moment to (x, y), turning smoothly to the heading.
     * Positions in inches from the start (x forward, y left), heading in degrees (left turn positive).
     * maxSpeed is 0 to 1 (1 = full speed).
     */
    public static FollowPathCommand goTo(PedroDriveSubsystem drive, double x, double y,
                                         double headingDegrees, double maxSpeed) {
        Pose target = new Pose(x, y, Math.toRadians(headingDegrees));
        return new FollowPathCommand(drive, () -> {
            Pose from = drive.getFollower().pose();
            if (from.distance(target) < 1.0) {
                return null;  // already there (a zero-length line isn't a real path)
            }
            return line(from, target)
                    .linear(from, target)
                    .with(PedroConstants.foresightConfig.maxPathSpeed.at(maxSpeed));
        }) {
            @Override
            public void initialize() {
                super.initialize();
                if (isAlreadyThere()) {
                    drive.getFollower().hold(target);  // just turn to the heading and stay
                }
            }
        };
    }

    @Override
    public void initialize() {
        Path path = pathMaker.get();
        nothingToDo = path == null;
        if (!nothingToDo) {
            drive.getFollower().follow(path);
        }
    }

    protected boolean isAlreadyThere() {
        return nothingToDo;
    }

    @Override
    public boolean isFinished() {
        return nothingToDo || !drive.getFollower().following();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.holdHere();  // timed out or cancelled: stop where it is instead of finishing the path
        }
    }
}
