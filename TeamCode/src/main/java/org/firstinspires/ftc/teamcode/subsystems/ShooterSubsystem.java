package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

/*
 * The flywheel shooter: one motor that spins the flywheel.
 * Name in the robot configuration:  "shooter"
 *
 * HOW THE SPEED IS CONTROLLED (our own simple controller, not the hub's built-in one):
 *   1. BASE POWER: target RPM / the motor's top RPM. For 1000 RPM on a 6000 RPM motor that's 0.17.
 *      This alone gets the flywheel close to the right speed, with no sensor at all.
 *   2. SMALL CORRECTION (only if the encoder is working): if the wheel is slower than the target,
 *      add a little power; if faster, take a little away. A slow "catch-up" part also builds up
 *      over time to close any small steady gap. The whole correction is LIMITED to
 *      MAX_CORRECTION, so the flywheel can never run away to full speed.
 *   3. SAFETY CHECK: if the encoder reads nothing or reads backwards while the motor is running,
 *      the correction is switched off and it just uses the base power. The Driver Hub says so.
 *
 * Why: the hub's built-in speed control keeps adding power until the encoder reading matches.
 * With a missing, unplugged or backwards encoder reading it never matches, so the flywheel
 * kept speeding up no matter what SHOOT_RPM was set to.
 *
 * Settings used from Constants.ShooterConstants: SHOOT_RPM, USE_ENCODER, READY_TOLERANCE_RPM, SPIN_UP_MS.
 * The motor's top RPM comes from the motor type picked in the robot configuration,
 * so pick the exact goBILDA motor there (the RPM is printed on the motor's label).
 *
 * Get it with ShooterSubsystem.getInstance().
 */
public class ShooterSubsystem extends SubsystemBase {

    // ---- Tuning for the speed controller ----
    private static final double CORRECTION_GAIN = 0.0004;  // extra power per RPM of error (0.0004 = +0.04 at 100 RPM slow)
    private static final double CATCH_UP_GAIN = 0.0003;    // how fast the slow "catch-up" part builds (per RPM per second)
    private static final double MAX_CORRECTION = 0.20;     // the correction can never add or remove more than this
    private static final double FALLBACK_MAX_RPM = 6000;   // used if the configuration doesn't give the motor's top RPM
    private static final long ENCODER_CHECK_MS = 750;      // how long after starting before checking the encoder
    // -------------------------------------------

    // ---- One shared copy of this subsystem (same idea as getInstance() in the Astrobots code) ----
    private static ShooterSubsystem instance = null;

    /** Gets the robot's one ShooterSubsystem, creating it the first time. */
    public static synchronized ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    /** Forgets the old copy. Bot calls this at Init so every OpMode starts with fresh hardware. */
    public static synchronized void clearInstance() {
        instance = null;
    }
    // ---------------------------------------------------------------------------------------------

    private final DcMotorEx shooter;   // null if not in the configuration
    private final double motorMaxRpm;

    private boolean on = false;
    private double targetRpm = 0;
    private double power = 0;
    private boolean encoderProblem = false;
    private double catchUp = 0;
    private final ElapsedTime timeOn = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private ShooterSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");

        double maxRpm = FALLBACK_MAX_RPM;
        if (shooter != null) {
            // If the flywheel spins the wrong way, change FORWARD to REVERSE.
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            // RUN_WITHOUT_ENCODER = WE set the power. The hub's own speed control is OFF.
            // (The encoder can still be read for the speed.)
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT lets the heavy flywheel coast down when turned off instead of slamming to a stop.
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            double configured = shooter.getMotorType().getMaxRPM();
            if (configured > 0) {
                maxRpm = configured;
            }
        }
        motorMaxRpm = maxRpm;
        stop();
    }

    /** Spins the flywheel at this many RPM. 0 turns it off. */
    public void setTargetRpm(double rpm) {
        boolean turningOn = rpm > 0 && !on;
        on = rpm > 0;
        targetRpm = Math.max(0, rpm);
        if (turningOn) {
            timeOn.reset();
            loopTimer.reset();
            encoderProblem = false;
            catchUp = 0;
        }
        update();
    }

    public void stop() {
        setTargetRpm(0);
    }

    // Runs automatically every loop: works out the power for this loop.
    @Override
    public void periodic() {
        update();
    }

    private void update() {
        if (shooter == null) {
            return;
        }
        if (!on) {
            power = 0;
            shooter.setPower(0);   // just let it coast
            return;
        }

        // 1. Base power from the target speed.
        double basePower = targetRpm / motorMaxRpm;
        double newPower = basePower;

        if (ShooterConstants.USE_ENCODER) {
            double rpm = getRpm();

            // 3. Safety check, once the flywheel has had time to start turning.
            if (timeOn.milliseconds() > ENCODER_CHECK_MS
                    && (rpm < -50 || Math.abs(rpm) < targetRpm * 0.05)) {
                encoderProblem = true;   // stays off until the flywheel is turned off and on again
            }

            // 2. Small, limited correction.
            if (!encoderProblem) {
                double error = targetRpm - rpm;
                double dt = Math.min(loopTimer.seconds(), 0.1);
                catchUp = clip(catchUp + error * CATCH_UP_GAIN * dt, MAX_CORRECTION);
                double correction = clip(error * CORRECTION_GAIN + catchUp, MAX_CORRECTION);
                newPower = basePower + correction;
            }
        }

        loopTimer.reset();
        power = Math.max(0, Math.min(1, newPower));
        shooter.setPower(power);
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    public boolean isOn() {
        return on;
    }

    /** How fast the flywheel is really spinning, in RPM (needs the encoder cable). */
    public double getRpm() {
        return shooter == null ? 0 : shooter.getVelocity() / ticksPerRev() * 60.0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** True when the flywheel is on and ready to shoot. */
    public boolean isAtSpeed() {
        if (!on || shooter == null) {
            return false;
        }
        if (!ShooterConstants.USE_ENCODER || encoderProblem) {
            // Can't trust the speed reading, so just give it time to spin up.
            return timeOn.milliseconds() >= ShooterConstants.SPIN_UP_MS;
        }
        return Math.abs(getRpm() - targetRpm) <= ShooterConstants.READY_TOLERANCE_RPM;
    }

    /** One line for the Driver Hub that says what the shooter is doing, or what's wrong. */
    public String getStatus() {
        if (shooter == null) {
            return "NOT FOUND - name the flywheel motor \"shooter\" in the configuration";
        }
        if (!on) {
            return "off (driver 2: right bumper)";
        }
        String ready = isAtSpeed() ? "READY" : "spinning up";
        String powerText = "power " + Math.round(power * 100) + "%";
        if (!ShooterConstants.USE_ENCODER) {
            return ready + "  " + powerText + " (no encoder)";
        }
        String speed = Math.round(getRpm()) + " / " + Math.round(targetRpm) + " RPM";
        if (encoderProblem) {
            return ready + "  " + powerText + "  ENCODER PROBLEM (reads " + Math.round(getRpm())
                    + " RPM) - check the encoder cable is in the SAME port number as the motor";
        }
        return ready + "  " + speed + "  " + powerText;
    }

    // Encoder counts per motor turn, from the motor type picked in the configuration.
    private double ticksPerRev() {
        double ticks = shooter.getMotorType().getTicksPerRev();
        return ticks > 0 ? ticks : 28.0;  // 28 = a bare goBILDA motor, as a safe fallback
    }
}
