package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

/*
 * Pedro Pathing's Follower, wrapped as an FTCLib subsystem so it fits our command structure.
 * Only the Pedro autonomous uses this. It drives the four drive motors itself, so while it
 * exists, nothing else should set drive motor powers (aim through it with PedroAimCommand).
 *
 * Positions are in inches measured from where the robot starts, same as Constants.AutoConstants:
 *   x = forward, y = to the LEFT, heading 0 = the way the robot faced at the start, left turn positive.
 *
 * Get it with PedroDriveSubsystem.getInstance().
 */
public class PedroDriveSubsystem extends SubsystemBase {

    // ---- One shared copy of this subsystem (same idea as getInstance() in the Astrobots code) ----
    private static PedroDriveSubsystem instance = null;

    public static synchronized PedroDriveSubsystem getInstance() {
        if (instance == null) {
            instance = new PedroDriveSubsystem();
        }
        return instance;
    }

    public static synchronized void clearInstance() {
        instance = null;
    }

    /** The current copy, or null if nothing has created one (so telemetry doesn't create it by accident). */
    public static synchronized PedroDriveSubsystem getIfCreated() {
        return instance;
    }
    // ---------------------------------------------------------------------------------------------

    private final Follower follower;

    private PedroDriveSubsystem() {
        follower = PedroConstants.create(Constants.hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
    }

    public Follower getFollower() {
        return follower;
    }

    /** Runs every loop: updates the position and the drive powers. */
    @Override
    public void periodic() {
        follower.update();
    }

    /** Makes the robot's current spot (0, 0) facing heading 0. Call right when autonomous starts. */
    public void resetPose() {
        follower.setPose(new Pose(0, 0, 0));
        follower.hold(new Pose(0, 0, 0));
    }

    /** Stay where the robot is right now (it will push back if bumped). */
    public void holdHere() {
        follower.hold(follower.pose());
    }

    /** Stop driving completely. */
    public void stop() {
        follower.stop();
    }

    public double getX() { return follower.pose().x(); }
    public double getY() { return follower.pose().y(); }
    public double getHeadingDegrees() { return Math.toDegrees(follower.pose().heading()); }
    public String getMode() { return follower.mode().toString(); }
}
