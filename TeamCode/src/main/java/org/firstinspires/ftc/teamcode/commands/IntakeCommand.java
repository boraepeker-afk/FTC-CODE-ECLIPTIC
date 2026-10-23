package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/*
 * Runs the intake from the gamepad triggers, the same way goBILDA's StarterBot code does:
 *   right trigger = pull POLLEN in, left trigger = push it back out.
 * The further you pull a trigger, the faster the intake spins.
 * Pulling both triggers at once cancels out and the intake stops.
 *
 * This is the intake's default command, so it runs the whole time unless another
 * command (like a future launcher command) takes over the intake.
 */
public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final DoubleSupplier inTrigger;
    private final DoubleSupplier outTrigger;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier inTrigger, DoubleSupplier outTrigger) {
        this.intake = intake;
        this.inTrigger = inTrigger;
        this.outTrigger = outTrigger;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPower(inTrigger.getAsDouble() - outTrigger.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
