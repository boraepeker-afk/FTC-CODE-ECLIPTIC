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
 *
 * Name in the robot configuration:  "shooter"
 * If it isn't in the configuration, the robot still runs and the screen says so.
 *
 * Two ways to run it (pick with ShooterConstants.USE_ENCODER):
 *   true  - VELOCITY control: holds an exact speed in RPM, so shots are consistent.
 *           Needs the motor's ENCODER cable plugged into the hub next to its power port.
 *   false - plain POWER: just runs the motor at SHOOT_POWER (0 to 1). Use this to test
 *           the shooter if the encoder cable isn't plugged in or isn't working.
 *
 * Get it with ShooterSubsystem.getInstance().
 */
public class ShooterSubsystem extends SubsystemBase {

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

    private boolean on = false;
    private double targetRpm = 0;
    private final ElapsedTime timeOn = new ElapsedTime();

    private ShooterSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        shooter = hardwareMap.tryGet(DcMotorEx.class, "shooter");

        if (shooter != null) {
            // If the flywheel spins the wrong way, change FORWARD to REVERSE.
            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter.setMode(ShooterConstants.USE_ENCODER
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // FLOAT lets the heavy flywheel coast down when turned off instead of slamming to a stop.
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        stop();
    }

    /** Spins the flywheel up to this many RPM (or SHOOT_POWER without the encoder). 0 turns it off. */
    public void setTargetRpm(double rpm) {
        boolean turningOn = rpm > 0 && !on;
        on = rpm > 0;
        targetRpm = rpm;
        if (turningOn) {
            timeOn.reset();
        }
        if (shooter == null) {
            return;
        }
        if (!on) {
            shooter.setPower(0);   // just let it coast
        } else if (ShooterConstants.USE_ENCODER) {
            shooter.setVelocity(rpm / 60.0 * ticksPerRev());
        } else {
            shooter.setPower(ShooterConstants.SHOOT_POWER);
        }
    }

    public void stop() {
        setTargetRpm(0);
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
        if (!ShooterConstants.USE_ENCODER) {
            // Without the encoder we can't measure the speed, so just give it time to spin up.
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
        if (!ShooterConstants.USE_ENCODER) {
            return (isAtSpeed() ? "READY" : "spinning up") + " - power "
                    + Math.round(ShooterConstants.SHOOT_POWER * 100) + "% (no encoder)";
        }
        String speed = Math.round(getRpm()) + " / " + Math.round(targetRpm) + " RPM";
        if (isAtSpeed()) {
            return "READY  " + speed;
        }
        if (timeOn.milliseconds() > 1500 && Math.abs(getRpm()) < 50) {
            // Told to spin for 1.5 s but the encoder still reads nothing.
            return "speed reads 0 - check the ENCODER cable, or set USE_ENCODER = false  " + speed;
        }
        return "spinning up  " + speed;
    }

    // Encoder counts per motor turn. It comes from the motor type picked in the configuration,
    // so pick the right goBILDA motor there.
    private double ticksPerRev() {
        double ticks = shooter.getMotorType().getTicksPerRev();
        return ticks > 0 ? ticks : 28.0;  // 28 = a bare goBILDA motor, as a safe fallback
    }
}
