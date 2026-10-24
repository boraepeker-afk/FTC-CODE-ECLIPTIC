package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Everything shared across the whole robot code, in one place:
 *   - the four "global" objects every file can use (hardwareMap, telemetry, pilot, copilot)
 *   - all the numbers you tune on the real robot, grouped by subsystem
 *
 * hardwareMap, telemetry, pilot and copilot are null until an OpMode calls Constants.init(this)
 * at Init, because they don't exist until the robot is turned on and an OpMode starts.
 */
public class Constants {

    public static HardwareMap hardwareMap = null;
    public static Telemetry telemetry = null;
    public static GamepadEx pilot = null;    // driver 1 (gamepad 1): drives
    public static GamepadEx copilot = null;  // driver 2 (gamepad 2): runs the intake and the shooter

    /** Connects the globals above to this OpMode's real hardware map, telemetry and gamepads. */
    public static void init(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        pilot = new GamepadEx(opMode.gamepad1);
        copilot = new GamepadEx(opMode.gamepad2);
    }

    public static class DriveConstants {
        // true = field-centric (forward = away from the driver), false = robot-centric (forward = robot's front)
        public static final boolean FIELD_CENTRIC = false;
        public static final double NORMAL_SPEED = 1.0;
        public static final double SLOW_SPEED = 0.4;          // while driver 1 holds the right bumper
        public static final double STRAFE_CORRECTION = 1.1;   // strafing is less efficient; this compensates

        // ---- Position tracking with the drive motor encoders ----
        public static final double WHEEL_DIAMETER_INCHES = 104 / 25.4;  // goBILDA 104 mm mecanum wheels
        public static final double GEAR_RATIO = 1.0;  // wheel turns per motor turn (1.0 = wheel right on the motor)
        // Mecanum wheels slip sideways, so strafing covers less distance than the encoders think.
        // Tune: strafe the robot 48 inches with a tape measure; if the screen says 40, use 48/40 = 1.2.
        public static final double STRAFE_DISTANCE_MULTIPLIER = 1.0;
    }

    // How DriveToPoseCommand drives to a spot.
    public static class PathConstants {
        public static final double DRIVE_GAIN = 0.05;          // power per inch away (0.05 = full speed at 20 in)
        public static final double MIN_POWER = 0.06;           // small push so it doesn't stop just short
        public static final double TURN_GAIN = 0.015;          // turning power per degree off
        public static final double MAX_TURN = 0.5;
        public static final double POSITION_TOLERANCE = 1.5;   // inches: "close enough"
        public static final double HEADING_TOLERANCE = 3.0;    // degrees: "close enough"
    }

    public static class VisionConstants {
        // The webcam's name in the robot configuration.
        public static final String CAMERA_NAME = "Webcam 1";
        // 1 = sees tags farthest away but slowest, 3 = fastest but only sees tags up close.
        public static final int DECIMATION = 2;
    }

    // Driver 1 holding the left bumper in TeleOp: the robot turns to face the target.
    public static class AimConstants {
        public static final double TURN_GAIN = 0.01;    // turning power per degree off-center (raise = faster, lower = less wobble)
        public static final double MAX_AIM_TURN = 0.3;  // the most turning power aiming may use
        public static final double AIM_SPEED = 0.5;     // driving speed while aiming
    }

    public static class ShooterConstants {
        // true = hold an exact speed (needs the motor's encoder cable plugged in).
        // false = just run at SHOOT_POWER. Use false to test if the shooter won't run.
        public static final boolean USE_ENCODER = true;
        public static final double SHOOT_POWER = 0.8;   // only used when USE_ENCODER is false (0 to 1)
        public static final long SPIN_UP_MS = 1500;     // without the encoder: how long before "READY"

        // Flywheel speed while shooting, in RPM. Start here and raise it if shots fall short,
        // lower it if they fly too far. (A bare goBILDA 6000 RPM motor tops out near 6000.)
        public static final double SHOOT_RPM = 3000;
        // How close to SHOOT_RPM counts as "READY" on the Driver Hub screen.
        public static final double READY_TOLERANCE_RPM = 150;
    }

    /*
     * AUTONOMOUS. 30 seconds. Points: LEAVE the wall (3), tip the HIVE by launching into its
     * upward Cell (20), PARK in your Loading Zone (5).
     *
     * ALL SPOTS ARE MEASURED FROM WHERE THE ROBOT STARTS, written as {x, y, heading}:
     *   x       = inches FORWARD from the starting spot
     *   y       = inches to the LEFT (use negative numbers for right)
     *   heading = degrees to face. 0 = the way the robot faced at the start, 90 = turned left, -90 = turned right
     * Write them for the BLUE side. On RED, the code mirrors left/right automatically (MIRROR_FOR_RED).
     * Measure them on a real field with a tape measure, then test and adjust.
     */
    public static class AutoConstants {
        // ---- Spots (BLUE side; placeholders - MEASURE YOURS) ----
        public static final double[] SHOOT_POSE         = {30, 0, 0};    // where to shoot from, facing the HIVE
        public static final double[] COLLECT_START_POSE = {20, 30, 90};  // lined up at the start of the row of POLLEN
        public static final double[] COLLECT_END_POSE   = {20, 54, 90};  // end of the row (drives slowly through it)
        public static final double[] PARK_POSE          = {12, 12, 0};   // inside your Loading Zone

        // true = on RED, flip every y and heading (mirror image of the blue path).
        public static final boolean MIRROR_FOR_RED = true;

        // ---- Speeds ----
        public static final double DRIVE_SPEED = 0.6;      // normal driving between spots
        public static final double COLLECT_SPEED = 0.3;    // driving slowly through the POLLEN with the intake on
        public static final double INTAKE_POWER = 1.0;     // intake while collecting

        // ---- Timeouts: the most time each step may take (milliseconds) ----
        public static final long DRIVE_TIMEOUT_MS = 4000;
        public static final long COLLECT_TIMEOUT_MS = 4000;
        public static final long AIM_TIMEOUT_MS = 1500;    // camera fine-aim at the shooting spot
        public static final long SPIN_UP_TIMEOUT_MS = 2500; // longest to wait for the flywheel
        public static final long FEED_MS = 2500;           // intake time to shoot all the balls

        // Only go collect more POLLEN if fewer than this many seconds of auto have gone by.
        // (The collect trip + second shot + park takes roughly 14 s.)
        public static final double COLLECT_CUTOFF_S = 14;
        // Skip ahead and park once this many seconds have gone by.
        public static final double PARK_BY_S = 26;

        // ---- Shooting ----
        public static final double AUTO_SHOOT_RPM = ShooterConstants.SHOOT_RPM;
        public static final double FEED_POWER = 1.0;

        // ---- Camera fine-aim (after the encoders get close) ----
        public static final boolean USE_CAMERA_AIM = true;
        public static final double DESIRED_DISTANCE = 24.0;  // inches from the camera to the target when shooting
        public static final double SPEED_GAIN  = 0.02;
        public static final double STRAFE_GAIN = 0.015;
        public static final double TURN_GAIN   = 0.01;
        public static final double MAX_AUTO_SPEED  = 0.5;
        public static final double MAX_AUTO_STRAFE = 0.5;
        public static final double MAX_AUTO_TURN   = 0.3;
        public static final double RANGE_TOLERANCE = 2.0;    // inches
        public static final double ANGLE_TOLERANCE = 3.0;    // degrees

        // Used by SearchForTargetCommand (not part of the main routine any more).
        public static final double SEARCH_TURN = 0.2;
    }
}
