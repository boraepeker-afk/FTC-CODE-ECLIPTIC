package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/*
 * The intake subsystem for the goBILDA BIOBUZZ StarterBot intake:
 *   - one motor that spins the intake roller (the Gecko wheels)
 *   - two continuous-rotation servos that pull POLLEN out of corners
 * All three run together at the same power.
 *
 * The names below are the ones goBILDA's StarterBot code uses, so they must match
 * your Driver Station configuration exactly:
 *   "intake"             - motor
 *   "left_intake_servo"  - continuous rotation servo
 *   "right_intake_servo" - continuous rotation servo
 *
 * Any part that isn't in the configuration yet is simply skipped, so the robot
 * can still drive before the intake is built.
 * Get it with IntakeSubsystem.getInstance().
 */
public class IntakeSubsystem extends SubsystemBase {

    // ---- One shared copy of this subsystem (same idea as getInstance() in the Astrobots code) ----
    private static IntakeSubsystem instance = null;

    /** Gets the robot's one IntakeSubsystem, creating it the first time. */
    public static synchronized IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    /** Forgets the old copy. Bot calls this at Init so every OpMode starts with fresh hardware. */
    public static synchronized void clearInstance() {
        instance = null;
    }
    // ---------------------------------------------------------------------------------------------

    private final DcMotor intakeMotor;        // null if not in the configuration
    private final CRServo leftIntakeServo;    // null if not in the configuration
    private final CRServo rightIntakeServo;   // null if not in the configuration

    private double power = 0;

    private IntakeSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        intakeMotor      = hardwareMap.tryGet(DcMotor.class, "intake");
        leftIntakeServo  = hardwareMap.tryGet(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.tryGet(CRServo.class, "right_intake_servo");

        // The two servos face opposite ways, so one is reversed to make both pull POLLEN in.
        // If a servo or the roller spins the wrong way, flip it between FORWARD and REVERSE.
        if (intakeMotor != null) {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            // Stop the roller right away when the intake is turned off.
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (leftIntakeServo != null) {
            leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (rightIntakeServo != null) {
            rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        stop();
    }

    /**
     * Runs the roller and both servos (whichever of them are on the robot).
     *
     * @param power from -1 to 1: positive pulls POLLEN in, negative pushes it back out
     */
    public void setPower(double power) {
        this.power = Math.max(-1.0, Math.min(1.0, power));
        if (intakeMotor != null)      intakeMotor.setPower(this.power);
        if (leftIntakeServo != null)  leftIntakeServo.setPower(this.power);
        if (rightIntakeServo != null) rightIntakeServo.setPower(this.power);
    }

    public void stop() {
        setPower(0);
    }

    /** Current intake power, for telemetry. */
    public double getPower() {
        return power;
    }

    /** Lists which intake parts were found, for telemetry, e.g. "motor, left servo". */
    public String getPartsFound() {
        StringBuilder found = new StringBuilder();
        if (intakeMotor != null)      found.append("motor");
        if (leftIntakeServo != null)  found.append(found.length() > 0 ? ", " : "").append("left servo");
        if (rightIntakeServo != null) found.append(found.length() > 0 ? ", " : "").append("right servo");
        return found.length() > 0 ? found.toString() : "none (not in the configuration yet)";
    }
}
