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
 * The drivetrain subsystem: the only class that touches the four drive motors.
 * Commands tell it what to do by calling drive(), driveFieldCentric() and stop().
 *
 * It also keeps track of WHERE the robot is and WHICH WAY IT FACES (a "localizer"), using the
 * encoders inside the four drive motors. For heading it uses the Control Hub's IMU if the
 * configuration has one named "imu" (more accurate); if not, it works it out from the wheel
 * encoders instead, so the robot still runs. Position is measured in inches from where
 * the robot was when the position was last reset (start of autonomous):
 *   x = forward from the start, y = to the LEFT of the start, heading = degrees, left turn is positive.
 * This needs the drive motors' encoder cables plugged in.
 * Get it with DriveSubsystem.getInstance().
 *
 * The four motor names below must match your Driver Station configuration exactly.
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

    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;

    // ---- Position tracking (localizer) ----
    private final double ticksPerInch;
    private double x = 0, y = 0;            // inches
    private double heading = 0;             // radians, left turn positive
    private final IMU imu;                  // null if there's no "imu" in the configuration
    private double imuOffsetRad = 0;        // IMU reading that counts as heading 0
    private int lastFL, lastBL, lastFR, lastBR;

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

        // Zero the wheel encoders. The motors still run on plain power (RUN_WITHOUT_ENCODER),
        // the encoders are only read to track position.
        for (DcMotor motor : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        ticksPerInch = calculateTicksPerInch();

        // The IMU is optional. If it's in the configuration, use it for heading.
        IMU foundImu = hardwareMap.tryGet(IMU.class, "imu");
        if (foundImu != null) {
            try {
                foundImu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                        DriveConstants.HUB_LOGO_DIRECTION, DriveConstants.HUB_USB_DIRECTION)));
                foundImu.resetYaw();
            } catch (RuntimeException e) {
                foundImu = null;  // couldn't start it (for example a bad orientation setting): use the encoders
            }
        }
        imu = foundImu;
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

    /** Makes the direction the robot is facing right now count as "forward" (heading 0). */
    public void resetHeading() {
        heading = 0;
        if (imu != null) {
            imuOffsetRad = readImuRadians();
        }
    }

    /** Where the heading comes from, for the Driver Hub screen. */
    public String getHeadingSource() {
        return imu != null ? "IMU" : "wheel encoders (no \"imu\" in the configuration)";
    }

    private double readImuRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Which way the robot faces, in radians, from the wheel encoders. Turning left is positive. */
    public double getHeadingRadians() {
        return heading;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
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

        double dFL = (fl - lastFL) / ticksPerInch;
        double dBL = (bl - lastBL) / ticksPerInch;
        double dFR = (fr - lastFR) / ticksPerInch;
        double dBR = (br - lastBR) / ticksPerInch;
        lastFL = fl; lastBL = bl; lastFR = fr; lastBR = br;

        // Mecanum math: how far the robot moved forward and to the right, in its own view.
        double forward = (dFL + dBL + dFR + dBR) / 4.0;
        double right   = (dFL - dBL - dFR + dBR) / 4.0 * DriveConstants.STRAFE_DISTANCE_MULTIPLIER;

        // How much it turned: the right wheels go forward and the left wheels go back on a left turn.
        // The robot's width and length set how far the wheels roll for each degree of turning.
        double turn = (dFR + dBR - dFL - dBL)
                / (2.0 * (DriveConstants.TRACK_WIDTH_INCHES + DriveConstants.WHEEL_BASE_INCHES));

        // With the IMU, use its (more accurate) reading for how much the robot turned.
        if (imu != null) {
            turn = angleWrap(angleWrap(readImuRadians() - imuOffsetRad) - heading);
        }

        // Turn that into field directions using the average heading during this loop.
        double h = heading + turn / 2.0;
        x += forward * Math.cos(h) + right * Math.sin(h);
        y += forward * Math.sin(h) - right * Math.cos(h);
        heading = angleWrap(heading + turn);
    }

    /** Makes the robot's current spot (0, 0) facing heading 0. Call at the start of autonomous. */
    public void resetPose() {
        lastFL = frontLeft.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        lastFR = frontRight.getCurrentPosition();
        lastBR = backRight.getCurrentPosition();
        resetHeading();
        x = 0;
        y = 0;
    }

    /** Inches forward from the starting spot. */
    public double getX() { return x; }

    /** Inches to the LEFT of the starting spot. */
    public double getY() { return y; }

    /** Degrees turned since the starting spot. Left (counterclockwise) is positive. */
    public double getPoseHeadingDegrees() {
        return Math.toDegrees(heading);
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
