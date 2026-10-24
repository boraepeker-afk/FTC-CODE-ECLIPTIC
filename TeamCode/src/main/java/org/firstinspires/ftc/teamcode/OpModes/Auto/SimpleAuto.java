package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.SimpleAutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/*
 * BACKUP AUTONOMOUS - no encoders, no camera, no IMU needed. Everything is timed.
 *   1. Flywheel on, drive forward for DRIVE_MS (leaves the wall = 3 points).
 *   2. Wait for the flywheel, then run the intake to shoot the preloaded POLLEN.
 *   3. Stop everything.
 * Numbers are in Constants.SimpleAutoConstants. Line the robot up aimed at the HIVE before Start.
 */
@Autonomous(name = "Simple Auto (backup)")
public class SimpleAuto extends CommandOpMode {

    private Bot bot;
    private String step = "waiting for Start";

    @Override
    public void initialize() {
        reset();
        Constants.init(this);
        bot = new Bot(Bot.OpMode.AUTO);

        DriveSubsystem drive = DriveSubsystem.getInstance();
        IntakeSubsystem intake = IntakeSubsystem.getInstance();
        ShooterSubsystem shooter = ShooterSubsystem.getInstance();

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> step = "1. start delay"),
                new WaitCommand(SimpleAutoConstants.START_DELAY_MS),

                new InstantCommand(() -> shooter.setTargetRpm(SimpleAutoConstants.SHOOT_RPM), shooter),
                new InstantCommand(() -> step = "2. driving forward"),
                new RunCommand(() -> drive.drive(SimpleAutoConstants.DRIVE_POWER, 0, 0, 1.0), drive)
                        .withTimeout(SimpleAutoConstants.DRIVE_MS),
                new InstantCommand(drive::stop, drive),

                new InstantCommand(() -> step = "3. waiting for the flywheel"),
                new WaitUntilCommand(shooter::isAtSpeed).withTimeout(SimpleAutoConstants.SPIN_UP_MS),
                new InstantCommand(() -> step = "4. shooting"),
                new RunCommand(() -> intake.setPower(SimpleAutoConstants.FEED_POWER), intake)
                        .withTimeout(SimpleAutoConstants.FEED_MS),

                new InstantCommand(intake::stop, intake),
                new InstantCommand(shooter::stop, shooter),
                new InstantCommand(drive::stop, drive),
                new InstantCommand(() -> step = "done")));

        telemetry.addData("Simple Auto", "drive forward %d ms, then shoot. Press Start.", SimpleAutoConstants.DRIVE_MS);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Step", step);
        telemetry.addData("Shooter", ShooterSubsystem.getInstance().getStatus());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            if (bot != null) {
                bot.close();
            }
        }
    }
}
