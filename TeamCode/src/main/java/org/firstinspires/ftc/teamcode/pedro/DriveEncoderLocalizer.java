package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/*
 * Tells Pedro Pathing where the robot is, using the encoders inside the four drive motors
 * (Pedro 3 has no built-in tracker for drive encoders, so this is ours).
 * Same math as DriveSubsystem: x = forward, y = to the LEFT, heading in radians, left turn positive.
 * Heading comes from the Control Hub IMU if the configuration has one named "imu"
 * (much more accurate), otherwise from the wheel encoders.
 *
 * To make it more accurate, tune these in Constants.DriveConstants (same as before):
 *   WHEEL_DIAMETER_INCHES, GEAR_RATIO, STRAFE_DISTANCE_MULTIPLIER,
 *   TRACK_WIDTH_INCHES and WHEEL_BASE_INCHES (only used without the IMU).
 * Check it with the AutoTune "Tests" -> "Localization Test" (see Tuning.java).
 *
 * Mecanum wheels slip, so this drifts more than odometry pods would. Keep paths short and simple.
 */
public class DriveEncoderLocalizer implements Localizer {

    private final DcMotor frontLeft, backLeft, frontRight, backRight;
    private final IMU imu;  // null if there's no "imu" in the configuration
    private final double ticksPerInch;

    private Pose pose = Pose.zero();
    private MotionState motionState = MotionState.ofVelocity(Pose.zero(), Velocity.zero());

    private int lastFL, lastBL, lastFR, lastBR;
    private double lastImuHeading;
    private long lastTimeNanos = 0;

    public DriveEncoderLocalizer(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, PedroConstants.FRONT_LEFT);
        backLeft   = hardwareMap.get(DcMotor.class, PedroConstants.BACK_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, PedroConstants.FRONT_RIGHT);
        backRight  = hardwareMap.get(DcMotor.class, PedroConstants.BACK_RIGHT);

        // Same directions as the drivetrain, so the encoders count up when driving forward.
        frontLeft.setDirection(PedroConstants.FRONT_LEFT_DIRECTION);
        backLeft.setDirection(PedroConstants.BACK_LEFT_DIRECTION);
        frontRight.setDirection(PedroConstants.FRONT_RIGHT_DIRECTION);
        backRight.setDirection(PedroConstants.BACK_RIGHT_DIRECTION);
        for (DcMotor motor : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Pedro sets the power; we only read the encoders
        }

        // Counts per motor turn come from the motor type picked in the robot configuration (as in DriveSubsystem).
        double ticksPerRev = frontLeft.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0) {
            ticksPerRev = 537.7;  // goBILDA 312 RPM motor, as a fallback
        }
        ticksPerInch = ticksPerRev * DriveConstants.GEAR_RATIO / (Math.PI * DriveConstants.WHEEL_DIAMETER_INCHES);

        IMU foundImu = hardwareMap.tryGet(IMU.class, "imu");
        if (foundImu != null) {
            try {
                foundImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                        DriveConstants.HUB_LOGO_DIRECTION, DriveConstants.HUB_USB_DIRECTION)));
            } catch (RuntimeException e) {
                foundImu = null;  // bad orientation setting or similar: use the encoders for heading
            }
        }
        imu = foundImu;

        setPose(Pose.zero());
    }

    /** True if heading comes from the IMU, false if from the wheel encoders. */
    public boolean usingImu() {
        return imu != null;
    }

    @Override
    public void setPose(Pose newPose) {
        pose = newPose;
        lastFL = frontLeft.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        lastFR = frontRight.getCurrentPosition();
        lastBR = backRight.getCurrentPosition();
        if (imu != null) {
            lastImuHeading = readImu();
        }
        lastTimeNanos = 0;
        motionState = MotionState.ofVelocity(pose, Velocity.zero());
    }

    @Override
    public void update() {
        int fl = frontLeft.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int br = backRight.getCurrentPosition();

        double dFL = (fl - lastFL) / ticksPerInch;
        double dBL = (bl - lastBL) / ticksPerInch;
        double dFR = (fr - lastFR) / ticksPerInch;
        double dBR = (br - lastBR) / ticksPerInch;
        lastFL = fl; lastBL = bl; lastFR = fr; lastBR = br;

        // How far the robot moved in its own view: forward, and to the LEFT (Pedro's direction).
        double forward = (dFL + dBL + dFR + dBR) / 4.0;
        double left = -(dFL - dBL - dFR + dBR) / 4.0 * DriveConstants.STRAFE_DISTANCE_MULTIPLIER;

        double turn;
        if (imu != null) {
            double now = readImu();
            turn = Angle.normalizeSigned(now - lastImuHeading);  // smallest change, -pi..pi
            lastImuHeading = now;
        } else {
            turn = (dFR + dBR - dFL - dBL)
                    / (2.0 * (DriveConstants.TRACK_WIDTH_INCHES + DriveConstants.WHEEL_BASE_INCHES));
        }

        // Turn the robot-view movement into field directions, using the average heading this loop.
        double h = pose.heading() + turn / 2.0;
        double dx = forward * Math.cos(h) - left * Math.sin(h);
        double dy = forward * Math.sin(h) + left * Math.cos(h);
        pose = new Pose(pose.x() + dx, pose.y() + dy, pose.heading() + turn);

        long nowNanos = System.nanoTime();
        Velocity velocity = Velocity.zero();
        if (lastTimeNanos != 0) {
            double dt = (nowNanos - lastTimeNanos) / 1e9;
            if (dt > 1e-4) {
                velocity = new Velocity(dx / dt, dy / dt, turn / dt);
            }
        }
        lastTimeNanos = nowNanos;
        motionState = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void reset() {
        setPose(Pose.zero());
    }

    private double readImu() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
