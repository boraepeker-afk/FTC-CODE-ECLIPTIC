package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * Spins the flywheel at a set speed for as long as this command runs,
 * then turns it off when the command ends.
 *
 * In TeleOp, driver 2's right bumper turns this on and off.
 */
public class ShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double rpm;

    public ShooterCommand(ShooterSubsystem shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRpm(rpm);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
