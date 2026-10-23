package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

/*
 * While this command runs, the robot turns by itself to face the AprilTag target the camera sees.
 * The driver still moves the robot with the left stick; only turning is automatic.
 * If no target is visible, the right stick turns the robot as usual.
 *
 * It's meant to run while a button is held, then hand control back to DriveCommand.
 */
public class AimAtTargetCommand extends CommandBase {

    // The tuning numbers are in Constants.AimConstants.
    private static final double TURN_GAIN = Constants.AimConstants.TURN_GAIN;
    private static final double MAX_AIM_TURN = Constants.AimConstants.MAX_AIM_TURN;
    private static final double AIM_SPEED = Constants.AimConstants.AIM_SPEED;

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final boolean fieldCentric;

    public AimAtTargetCommand(DriveSubsystem drive,
                              VisionSubsystem vision,
                              DoubleSupplier forward,
                              DoubleSupplier strafe,
                              DoubleSupplier turn,
                              boolean fieldCentric) {
        this.drive = drive;
        this.vision = vision;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.fieldCentric = fieldCentric;

        // Only the drivetrain is required: the camera keeps running for everything else.
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forwardPower = forward.getAsDouble() * AIM_SPEED;
        double strafePower = strafe.getAsDouble() * AIM_SPEED;

        double turnPower;
        if (vision.hasTarget()) {
            // Bearing is positive when the target is to the left; in drive(), positive turn is to the right.
            turnPower = clip(-vision.getTargetBearing() * TURN_GAIN, MAX_AIM_TURN);
        } else {
            turnPower = turn.getAsDouble() * AIM_SPEED;
        }

        if (fieldCentric) {
            drive.driveFieldCentric(forwardPower, strafePower, turnPower, 1.0);
        } else {
            drive.drive(forwardPower, strafePower, turnPower, 1.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
