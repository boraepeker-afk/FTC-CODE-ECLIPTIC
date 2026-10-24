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
        public static final boolean FIELD_CENTRIC = true;
        public static final double NORMAL_SPEED = 1.0;
        public static final double SLOW_SPEED = 0.4;          // while driver 1 holds the right bumper
        public static final double STRAFE_CORRECTION = 1.1;   // strafing is less efficient; this compensates
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

    public static class AutoConstants {
        // Search: turn in place until a tag is seen. Positive turns right, negative turns left.
        public static final double SEARCH_TURN = 0.2;
        public static final long SEARCH_TIMEOUT_MS = 4000;

        // Drive to the target.
        public static final double DESIRED_DISTANCE = 24.0;  // inches from the camera to the target when it stops
        public static final double SPEED_GAIN  = 0.02;       // 50% power at 25 inches of error
        public static final double STRAFE_GAIN = 0.015;      // 37% power at 25 degrees of yaw error
        public static final double TURN_GAIN   = 0.01;       // 25% power at 25 degrees of bearing error
        public static final double MAX_AUTO_SPEED  = 0.5;
        public static final double MAX_AUTO_STRAFE = 0.5;
        public static final double MAX_AUTO_TURN   = 0.3;
        public static final double RANGE_TOLERANCE = 2.0;    // inches
        public static final double ANGLE_TOLERANCE = 3.0;    // degrees
        public static final long DRIVE_TIMEOUT_MS = 8000;

        // Shooting after lining up with the target.
        public static final double AUTO_SHOOT_RPM = ShooterConstants.SHOOT_RPM;  // or a different number for auto
        public static final long SPIN_UP_TIMEOUT_MS = 3000;  // longest to wait for the flywheel to reach speed
        public static final long FEED_MS = 3000;             // how long to run the intake to feed the balls in
        public static final double FEED_POWER = 1.0;          // intake power while feeding (0 to 1)

        // If no target is found: drive straight forward for a moment instead.
        public static final long FALLBACK_DRIVE_MS = 1000;
        public static final double FALLBACK_POWER = 0.3;
    }
}
