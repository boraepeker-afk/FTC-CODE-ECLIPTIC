package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

/*
 * The flywheel shooter: one or two motors that spin the flywheel.
 *
 * It uses VELOCITY control: you ask for a speed in RPM and the Control Hub
 * adds or removes power by itself to hold that speed, even as the battery drains
 * or a game piece slows the wheel down. That makes shots land in the same place every time.
 * This needs the motor's ENCODER cable plugged into the hub next to the motor's power port.
 *
 * Names in the robot configuration:
 *   "shooter"         - flywheel motor
 *   "shooter_2"       - second flywheel motor (only if your flywheel has two)
 * Any motor that isn't in the configuration is skipped, so the robot still runs without it.
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

    private final DcMotorEx shooter;     // null if not in the configuration
    private final DcMotorEx shooter2;    // null if not in the configuration

    private double targetRpm = 0;

    private ShooterSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        shooter  = hardwareMap.tryGet(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.tryGet(DcMotorEx.class, "shooter_2");

        // If the flywheel spins backwards, flip that motor between FORWARD and REVERSE.
        // With two motors facing each other, one is usually REVERSE.
        setUp(shooter, DcMotorSimple.Direction.FORWARD);
        setUp(shooter2, DcMotorSimple.Direction.REVERSE);

        stop();
    }

    private static void setUp(DcMotorEx motor, DcMotorSimple.Direction direction) {
        if (motor == null) {
            return;
        }
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // FLOAT lets the heavy flywheel coast down when turned off instead of slamming to a stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Spins the flywheel up to this many RPM and holds it there. 0 turns it off. */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        setRpm(shooter, rpm);
        setRpm(shooter2, rpm);
    }

    private static void setRpm(DcMotorEx motor, double rpm) {
        if (motor == null) {
            return;
        }
        if (rpm == 0) {
            motor.setPower(0);   // just let it coast
        } else {
            motor.setVelocity(rpm / 60.0 * ticksPerRev(motor));
        }
    }

    public void stop() {
        setTargetRpm(0);
    }

    /** How fast the flywheel is really spinning, in RPM (0 if there's no shooter motor). */
    public double getRpm() {
        if (shooter != null) {
            return shooter.getVelocity() / ticksPerRev(shooter) * 60.0;
        }
        if (shooter2 != null) {
            return shooter2.getVelocity() / ticksPerRev(shooter2) * 60.0;
        }
        return 0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** True when the flywheel is on and close enough to its target speed to shoot. */
    public boolean isAtSpeed() {
        return targetRpm > 0
                && Math.abs(getRpm() - targetRpm) <= ShooterConstants.READY_TOLERANCE_RPM;
    }

    /** Lists which shooter motors were found, for telemetry. */
    public String getPartsFound() {
        if (shooter != null && shooter2 != null) return "2 motors";
        if (shooter != null || shooter2 != null) return "1 motor";
        return "none (not in the configuration yet)";
    }

    // Encoder counts per motor turn. It comes from the motor type you pick in the configuration,
    // so pick the right goBILDA motor there.
    private static double ticksPerRev(DcMotorEx motor) {
        double ticks = motor.getMotorType().getTicksPerRev();
        return ticks > 0 ? ticks : 28.0;  // 28 = a bare goBILDA motor, as a safe fallback
    }
}
