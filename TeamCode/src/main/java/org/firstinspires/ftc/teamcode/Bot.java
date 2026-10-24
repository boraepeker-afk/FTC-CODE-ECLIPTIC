package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Sets the robot up for TeleOp or Autonomous, so the OpMode files stay tiny.
 *   TELEOP: BotContainer connects the gamepads to commands.
 *   AUTO:   BotContainer's autonomous command is scheduled; it starts when you press Start.
 */
public class Bot extends Robot {

    public enum OpMode {
        TELEOP, AUTO
    }

    public final BotContainer container;
    private final OpMode opMode;

    public Bot(OpMode opMode) {
        this.opMode = opMode;

        // Start every OpMode with fresh subsystems connected to this OpMode's hardware.
        DriveSubsystem.clearInstance();
        IntakeSubsystem.clearInstance();
        ShooterSubsystem.clearInstance();
        VisionSubsystem.clearInstance();

        container = new BotContainer();

        switch (opMode) {
            case TELEOP:
                container.setDefaultCommands();
                container.configBindings();
                break;
            case AUTO:
                // Nothing yet: the drivers pick settings during Init, then the OpMode calls startAuto().
                break;
        }

        container.showInitTelemetry(opMode);
    }

    /** Schedules the autonomous routine with the drivers' settings. It runs once you press Start. */
    public void startAuto(AutoSettings settings) {
        schedule(container.getAutonomousCommand(settings));
    }

    /** Shows what the robot is doing on the Driver Hub. Call once per loop. */
    public void updateTelemetry() {
        container.showTelemetry(opMode);
    }

    /** Turns the camera off. Call when the OpMode ends. */
    public void close() {
        VisionSubsystem.getInstance().close();
    }
}
