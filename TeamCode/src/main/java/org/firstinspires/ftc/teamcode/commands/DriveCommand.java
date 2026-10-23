package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/*
 * Drives the robot from the gamepad. This is the drivetrain's default command,
 * so it runs the whole time unless another command takes over the drivetrain.
 *
 * It is given "suppliers" (small functions that return the current stick values)
 * so it reads fresh values from the gamepad every loop.
 */
public class DriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier slowMode;
    private final boolean fieldCentric;

    public DriveCommand(DriveSubsystem drive,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn,
                        BooleanSupplier slowMode,
                        boolean fieldCentric) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.slowMode = slowMode;
        this.fieldCentric = fieldCentric;

        // Tells the scheduler this command uses the drivetrain,
        // so two commands can never fight over the motors.
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double speed = slowMode.getAsBoolean()
                ? Constants.DriveConstants.SLOW_SPEED
                : Constants.DriveConstants.NORMAL_SPEED;

        if (fieldCentric) {
            drive.driveFieldCentric(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble(), speed);
        } else {
            drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble(), speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
