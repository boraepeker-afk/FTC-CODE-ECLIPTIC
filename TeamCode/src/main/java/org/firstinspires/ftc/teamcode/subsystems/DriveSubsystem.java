package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * The drivetrain subsystem: the only class that touches the four drive motors
 * and the Control Hub's built-in IMU (the sensor that knows which way the robot faces).
 * Commands tell it what to do by calling drive(), driveFieldCentric() and stop().
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
}
