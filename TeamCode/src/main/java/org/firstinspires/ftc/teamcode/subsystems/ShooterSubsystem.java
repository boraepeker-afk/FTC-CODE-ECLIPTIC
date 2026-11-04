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
 * HOW THE SPEED IS CONTROLLED (a PIDF controller we run ourselves, not the hub's built-in one):
 *   1. RAMP: when you turn it on, the target climbs smoothly up to SHOOT_RPM
 *      (RAMP_RPM_PER_SEC) instead of jumping there, so there is no sudden jolt.
 *   2. F (feedforward): kF * target gives the "base" power that should land close to the target.
 *   3. P, I, D (feedback, needs the encoder): compares the real speed to the target and adds or
 *      takes away a little power to close the gap. The I part is capped (MAX_I_POWER).
 *   4. SAFETY CHECK: if the encoder reads nothing or reads backwards while the motor is running,
 *      P, I and D are switched off and it just uses the F power. The Driver Hub says so.
 *
 * All the numbers to tune are in Constants.ShooterConstants. The motor's top RPM and encoder
 * ticks are written there too, instead of coming from the robot configuration, because a wrong
 * motor type in the configuration made the flywheel go straight to full speed.
 *
 * Get it with ShooterSubsystem.getInstance().
 */
public class ShooterSubsystem extends SubsystemBase {

    private static final long ENCODER_CHECK_MS = 750;  // how long after starting before checking the encoder

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
    private final double configuredMaxRpm;  // what the robot configuration says, only used for a warning

    private boolean on = false;
    private double targetRpm = 0;      // where we want to end up (SHOOT_RPM)
    private double rampedTarget = 0;   // where the ramp has got to so far
    private double power = 0;
    private boolean encoderProblem = false;

    // PID memory
    private double integral = 0;
    private double lastError = 0;
    private boolean firstLoop = true;

    private final ElapsedTime timeOn = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private ShooterSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");

        double configured = 0;
        if (shooter != null) {
            // If the flywheel spins the wrong way, change FORWARD to REVERSE.
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            // RUN_WITHOUT_ENCODER = WE set the power. The hub's own speed control is OFF.
            // (The encoder can still be read for the speed.)
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT lets the heavy flywheel coast down when turned off instead of slamming to a stop.
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            configured = shooter.getMotorType().getMaxRPM();
        }
        configuredMaxRpm = configured;
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
            rampedTarget = 0;
            integral = 0;
            lastError = 0;
            firstLoop = true;
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
            rampedTarget = 0;
            shooter.setPower(0);   // just let it coast
            return;
        }

        double dt = Math.min(loopTimer.seconds(), 0.1);  // seconds since last loop (capped so a hiccup can't cause a jump)
        loopTimer.reset();

        // Without the encoder: just a fixed power, as the Constants comment says.
        if (!ShooterConstants.USE_ENCODER) {
            setPower(ShooterConstants.SHOOT_POWER);
            return;
        }

        // 1. Ramp the target up (or down) smoothly.
        double maxStep = ShooterConstants.RAMP_RPM_PER_SEC * dt;
        if (rampedTarget < targetRpm) {
            rampedTarget = Math.min(targetRpm, rampedTarget + maxStep);
        } else {
            rampedTarget = Math.max(targetRpm, rampedTarget - maxStep);
        }

        // 2. Feedforward: the base power for this speed.
        double feedforward = ShooterConstants.kF * rampedTarget;

        double rpm = getRpm();

        // 4. Safety check, once the flywheel has had time to start turning.
        if (timeOn.milliseconds() > ENCODER_CHECK_MS
                && (rpm < -50 || Math.abs(rpm) < rampedTarget * 0.05)) {
            encoderProblem = true;   // stays on until the flywheel is turned off and on again
        }
        if (encoderProblem) {
            setPower(feedforward);
            return;
        }

        // 3. PID feedback.
        double error = rampedTarget - rpm;

        integral += error * dt;
        // Keep the I part from piling up forever ("integral windup").
        if (ShooterConstants.kI > 0) {
            double maxIntegral = ShooterConstants.MAX_I_POWER / ShooterConstants.kI;
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        }

        double derivative = (firstLoop || dt <= 0) ? 0 : (error - lastError) / dt;
        lastError = error;
        firstLoop = false;

        double feedback = ShooterConstants.kP * error
                + ShooterConstants.kI * integral
                + ShooterConstants.kD * derivative;
        // Cap the correction so a wrong speed reading can't push the flywheel to full power.
        feedback = Math.max(-ShooterConstants.MAX_FEEDBACK_POWER,
                Math.min(ShooterConstants.MAX_FEEDBACK_POWER, feedback));

        setPower(feedforward + feedback);
    }

    // Never negative (a flywheel shouldn't brake by spinning backwards) and never above MAX_POWER.
    private void setPower(double requested) {
        power = Math.max(0, Math.min(ShooterConstants.MAX_POWER, requested));
        shooter.setPower(power);
    }

    public boolean isOn() {
        return on;
    }

    /** How fast the flywheel is really spinning, in RPM (needs the encoder cable). */
    public double getRpm() {
        return shooter == null ? 0 : shooter.getVelocity() / ShooterConstants.TICKS_PER_REV * 60.0;
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
        return rampedTarget >= targetRpm
                && Math.abs(getRpm() - targetRpm) <= ShooterConstants.READY_TOLERANCE_RPM;
    }

    /** One line for the Driver Hub that says what the shooter is doing, or what's wrong. */
    public String getStatus() {
        if (shooter == null) {
            return "NOT FOUND - name the flywheel motor \"shooter\" in the configuration";
        }
        String warning = "";
        if (configuredMaxRpm > 0 && Math.abs(configuredMaxRpm - ShooterConstants.MOTOR_MAX_RPM) > 1) {
            warning = "  (note: configuration says a " + Math.round(configuredMaxRpm)
                    + " RPM motor, Constants says " + Math.round(ShooterConstants.MOTOR_MAX_RPM)
                    + " - make sure Constants matches the motor label)";
        }
        if (!on) {
            return "off (driver 2: right bumper)" + warning;
        }
        String ready = isAtSpeed() ? "READY" : "spinning up";
        String powerText = "power " + Math.round(power * 100) + "%";
        if (!ShooterConstants.USE_ENCODER) {
            return ready + "  " + powerText + " (no encoder)" + warning;
        }
        if (encoderProblem) {
            return ready + "  " + powerText + "  ENCODER PROBLEM (reads " + Math.round(getRpm())
                    + " RPM) - check the encoder cable is in the SAME port number as the motor" + warning;
        }
        String speed = Math.round(getRpm()) + " / " + Math.round(targetRpm) + " RPM";
        return ready + "  " + speed + "  " + powerText + warning;
    }
}
