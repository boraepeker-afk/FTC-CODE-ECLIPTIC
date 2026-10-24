package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/*
 * The drivetrain subsystem: the only class that touches the four drive motors
 * and the Control Hub's built-in IMU (the sensor that knows which way the robot faces).
 * Commands tell it what to do by calling drive(), driveFieldCentric() and stop().
 *
 * It also keeps track of WHERE the robot is (a "localizer"), using the encoders inside the
 * four drive motors plus the IMU for heading. Position is measured in inches from where the
 * robot was when the position was last reset (start of autonomous):
 *   x = forward from the start, y = to the LEFT of the start, heading = degrees, left turn is positive.
 * This needs the drive motors' encoder cables plugged in.
 * Get it with DriveSubsystem.getInstance().
 *
 * The four motor names below must match your Driver Station configuration exactly.
 * The IMU is named "imu" automatically when you scan the Control Hub.
 */
public class DriveSubsystem extends SubsystemBase {

    // ---- One shared copy of this subsystem (same idea as getInstance() in the Astrobots code) ----
    private static DriveSubsystem instance = null;

    /** Gets the robot's one DriveSubsystem, creating it the first time. */
    public static synchronized DriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }

    /** Forgets the old copy. Bot calls this at Init so every OpMode starts with fresh hardware. */
    public static synchronized void clearInstance() {
        instance = null;
    }
    // ---------------------------------------------------------------------------------------------

    // ---- SET THESE TO MATCH HOW YOUR CONTROL HUB IS MOUNTED ON THE ROBOT ----
    // Which way the REV logo on the Control Hub points: UP, DOWN, FORWARD, BACKWARD, LEFT or RIGHT.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    // Which way the Control Hub's USB ports point: UP, DOWN, FORWARD, BACKWARD, LEFT or RIGHT.
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    // ---------------------------------------------------------------------------

    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;
    private final IMU imu;

    // ---- Position tracking (localizer) ----
    private final double ticksPerInch;
    private double x = 0, y = 0;            // inches
    private double headingOffsetDeg = 0;    // so resetPose() can set heading to 0 without touching the IMU
    private int lastFL, lastBL, lastFR, lastBR;
    private double lastHeadingRad;

    private DriveSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // Motors on opposite sides face opposite ways, so one side is reversed.
        // If a wheel spins backwards when you push the stick forward,
        // flip that motor between FORWARD and REVERSE.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Stop quickly when the sticks are released instead of coasting.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));

        // Whichever way the robot faces when you press Init becomes "forward" on the field.
        imu.resetYaw();

        // Zero the wheel encoders. The motors still run on plain power (RUN_WITHOUT_ENCODER),
        // the encoders are only read to track position.
        for (DcMotor motor : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        ticksPerInch = calculateTicksPerInch();
        resetPose();
    }

    /**
     * Robot-centric mecanum drive: forward always means the robot's front.
     *
     * @param forward positive drives forward
     * @param strafe  positive slides right
     * @param turn    positive turns right
     * @param speed   overall speed limit from 0 to 1
     */
    public void drive(double forward, double strafe, double turn, double speed) {
        strafe *= Constants.DriveConstants.STRAFE_CORRECTION;  // strafing is less efficient; this compensates

        // Keeps every power within -1..1 while preserving the ratio between wheels.
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

        frontLeft.setPower((forward + strafe + turn) / denominator * speed);
        backLeft.setPower((forward - strafe + turn) / denominator * speed);
        frontRight.setPower((forward - strafe - turn) / denominator * speed);
        backRight.setPower((forward + strafe - turn) / denominator * speed);
    }

    /**
     * Field-centric mecanum drive: forward always means away from the driver,
     * no matter which way the robot is facing.
     *
     * @param forward positive drives away from the driver
     * @param strafe  positive slides to the driver's right
     * @param turn    positive turns right
     * @param speed   overall speed limit from 0 to 1
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double speed) {
        double heading = getHeadingRadians();

        // Rotate the stick direction by the opposite of the robot's heading,
        // turning "away from the driver" into the matching direction for the robot.
        double robotStrafe  = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double robotForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        drive(robotForward, robotStrafe, turn, speed);
    }

    /** Makes the direction the robot is facing right now count as "forward". */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** Which way the robot faces, in radians. Turning left (counterclockwise) is positive. */
    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // Current motor powers, for telemetry.
    public double getFrontLeftPower()  { return frontLeft.getPower(); }
    public double getBackLeftPower()   { return backLeft.getPower(); }
    public double getFrontRightPower() { return frontRight.getPower(); }
    public double getBackRightPower()  { return backRight.getPower(); }

    // =============================== Position tracking ===============================

    // Runs automatically every loop: adds up how far each wheel turned since last loop.
    @Override
    public void periodic() {
        int fl = frontLeft.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int br = backRight.getCurrentPosition();
        double heading = getPoseHeadingRadians();

        double dFL = (fl - lastFL) / ticksPerInch;
        double dBL = (bl - lastBL) / ticksPerInch;
        double dFR = (fr - lastFR) / ticksPerInch;
        double dBR = (br - lastBR) / ticksPerInch;
        lastFL = fl; lastBL = bl; lastFR = fr; lastBR = br;

        // Mecanum math: how far the robot moved forward and to the right, in its own view.
        double forward = (dFL + dBL + dFR + dBR) / 4.0;
        double right   = (dFL - dBL - dFR + dBR) / 4.0 * DriveConstants.STRAFE_DISTANCE_MULTIPLIER;

        // Turn that into field directions using the average heading during this loop.
        double h = lastHeadingRad + angleWrap(heading - lastHeadingRad) / 2.0;
        x += forward * Math.cos(h) + right * Math.sin(h);
        y += forward * Math.sin(h) - right * Math.cos(h);
        lastHeadingRad = heading;
    }

    /** Makes the robot's current spot (0, 0) facing heading 0. Call at the start of autonomous. */
    public void resetPose() {
        lastFL = frontLeft.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        lastFR = frontRight.getCurrentPosition();
        lastBR = backRight.getCurrentPosition();
        headingOffsetDeg = getHeadingDegrees();
        lastHeadingRad = 0;
        x = 0;
        y = 0;
    }

    /** Inches forward from the starting spot. */
    public double getX() { return x; }

    /** Inches to the LEFT of the starting spot. */
    public double getY() { return y; }

    /** Degrees turned since the starting spot. Left (counterclockwise) is positive. */
    public double getPoseHeadingDegrees() {
        return Math.toDegrees(angleWrap(Math.toRadians(getHeadingDegrees() - headingOffsetDeg)));
    }

    private double getPoseHeadingRadians() {
        return Math.toRadians(getPoseHeadingDegrees());
    }

    /** Keeps an angle between -180 and 180 degrees (in radians). */
    public static double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    // Encoder counts per inch of wheel travel. The counts per motor turn come from the motor type
    // picked in the robot configuration, so pick the right goBILDA motor there.
    private double calculateTicksPerInch() {
        double ticksPerRev = frontLeft.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0) {
            ticksPerRev = 537.7;  // goBILDA 312 RPM motor, as a fallback
        }
        double wheelCircumference = Math.PI * DriveConstants.WHEEL_DIAMETER_INCHES;
        return ticksPerRev * DriveConstants.GEAR_RATIO / wheelCircumference;
    }
}
