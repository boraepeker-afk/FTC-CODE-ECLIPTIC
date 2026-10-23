package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.commands.AimAtTargetCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToTargetCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.SearchForTargetCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Connects everything together:
 *   setDefaultCommands() - what each subsystem does when nothing else is using it
 *   configBindings()     - which gamepad buttons run which commands
 *   getAutonomousCommand() - the autonomous routine, step by step
 *
 * Driver 1 (pilot, gamepad 1) - driving:
 *   Left stick             - drive (field-centric: up = away from the driver)
 *   Right stick left/right - turn
 *   Hold right bumper      - slow mode
 *   Hold left bumper       - turn automatically to face the AprilTag target
 *   Back                   - reset "forward" to the way the robot faces now
 *
 * Driver 2 (copilot, gamepad 2) - intake and shooter:
 *   Right trigger          - intake in (pull harder = faster)
 *   Left trigger           - intake out
 *   Right bumper           - flywheel on / off (press once to start, again to stop)
 *                            Wait for "READY" on the Driver Hub, then feed with the right trigger.
 */
public class BotContainer {

    DriveSubsystem drive = DriveSubsystem.getInstance();
    VisionSubsystem vision = VisionSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    GamepadEx pilot = Constants.pilot;
    GamepadEx copilot = Constants.copilot;

    // Which autonomous step is running, for telemetry.
    private String autoStep = "waiting for Start";

    public void setDefaultCommands() {
        drive.setDefaultCommand(new DriveCommand(
                drive,
                () -> pilot.getLeftY(),   // GamepadEx already flips it so up is positive
                () -> pilot.getLeftX(),
                () -> pilot.getRightX(),
                () -> pilot.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                DriveConstants.FIELD_CENTRIC));

        intake.setDefaultCommand(new IntakeCommand(
                intake,
                () -> copilot.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> copilot.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
    }

    public void configBindings() {
        // Holding the left bumper aims at the target; letting go returns to normal driving.
        pilot.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new AimAtTargetCommand(
                        drive,
                        vision,
                        () -> pilot.getLeftY(),
                        () -> pilot.getLeftX(),
                        () -> pilot.getRightX(),
                        DriveConstants.FIELD_CENTRIC));

        // Pressing Back makes the way the robot faces right now count as "forward".
        pilot.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(drive::resetHeading));

        // Driver 2's right bumper turns the flywheel on, and pressing it again turns it off.
        copilot.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new ShooterCommand(shooter, ShooterConstants.SHOOT_RPM));

        // whenPressed runs a command once when the button goes down.
        // whenHeld runs it while the button is held and stops it when you let go.
        // toggleWhenPressed starts it on one press and stops it on the next.
    }

    /*
     * Autonomous:
     *   1. Turn slowly until the camera sees a target (gives up after SEARCH_TIMEOUT_MS).
     *   2. Found one: drive up to it and line up in front of it (gives up after DRIVE_TIMEOUT_MS).
     *      The flywheel turns on before driving, so it's up to speed by the time the robot lines up.
     *      Then wait for the flywheel to reach speed (at most SPIN_UP_TIMEOUT_MS)
     *      and run the intake for FEED_MS to feed the balls into the flywheel.
     *      Didn't: drive straight forward for a moment so the robot still leaves its spot.
     *   3. Stop everything.
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> autoStep = "1. searching for a target"),
                new SearchForTargetCommand(drive, vision).withTimeout(AutoConstants.SEARCH_TIMEOUT_MS),

                new ConditionalCommand(
                        // Found a target: drive to it and shoot.
                        new SequentialCommandGroup(
                                // Turn the flywheel on now so it's at speed by the time the robot lines up.
                                // It keeps spinning on its own until it's turned off below.
                                new InstantCommand(() -> shooter.setTargetRpm(AutoConstants.AUTO_SHOOT_RPM), shooter),

                                new InstantCommand(() -> autoStep = "2. driving to the target"),
                                new DriveToTargetCommand(drive, vision).withTimeout(AutoConstants.DRIVE_TIMEOUT_MS),
                                new InstantCommand(drive::stop, drive),

                                new InstantCommand(() -> autoStep = "3. waiting for the flywheel"),
                                new WaitUntilCommand(shooter::isAtSpeed).withTimeout(AutoConstants.SPIN_UP_TIMEOUT_MS),

                                new InstantCommand(() -> autoStep = "4. shooting"),
                                new RunCommand(() -> intake.setPower(AutoConstants.FEED_POWER), intake)
                                        .withTimeout(AutoConstants.FEED_MS),
                                new InstantCommand(intake::stop, intake),
                                new InstantCommand(shooter::stop, shooter)),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> autoStep = "2. no target found - driving forward"),
                                new RunCommand(() -> drive.drive(AutoConstants.FALLBACK_POWER, 0, 0, 1.0), drive)
                                        .withTimeout(AutoConstants.FALLBACK_DRIVE_MS)),
                        vision::hasTarget),

                new InstantCommand(drive::stop, drive),
                new InstantCommand(intake::stop, intake),
                new InstantCommand(shooter::stop, shooter),
                new InstantCommand(() -> autoStep = "done"));
    }

    // ---- Driver Hub screen ----

    public void showInitTelemetry(Bot.OpMode opMode) {
        Telemetry telemetry = Constants.telemetry;
        telemetry.addData("Camera", vision.isCameraConnected()
                ? "found" : "NOT found - name it \"" + Constants.VisionConstants.CAMERA_NAME + "\" in the configuration");
        if (opMode == Bot.OpMode.TELEOP) {
            telemetry.addData("Intake parts found", intake.getPartsFound());
            telemetry.addData("Shooter motors found", shooter.getPartsFound());
            telemetry.addData("Status", "Ready - point the robot away from you, then press Start");
        } else {
            telemetry.addData("Status", "Ready - press Start");
        }
        telemetry.update();
    }

    public void showTelemetry(Bot.OpMode opMode) {
        Telemetry telemetry = Constants.telemetry;

        if (opMode == Bot.OpMode.AUTO) {
            telemetry.addData("Step", autoStep);
            telemetry.addData("Shooter", "%.0f / %.0f RPM", shooter.getRpm(), shooter.getTargetRpm());
        } else {
            telemetry.addData("Mode", DriveConstants.FIELD_CENTRIC ? "Field-centric" : "Robot-centric");
            telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
        }

        if (!vision.isCameraConnected()) {
            telemetry.addData("Camera", "not found");
        } else if (vision.hasTarget()) {
            telemetry.addData("Target", vision.getTargetName());
            telemetry.addData("Range / Bearing / Yaw", "%.1f in / %.0f deg / %.0f deg",
                    vision.getTargetRange(), vision.getTargetBearing(), vision.getTargetYaw());
        } else {
            telemetry.addData("Target", "none");
        }

        if (opMode == Bot.OpMode.TELEOP) {
            telemetry.addData("Front L / R", "%.2f / %.2f", drive.getFrontLeftPower(), drive.getFrontRightPower());
            telemetry.addData("Back  L / R", "%.2f / %.2f", drive.getBackLeftPower(), drive.getBackRightPower());
            telemetry.addData("Slow mode", pilot.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? "ON" : "off");
            telemetry.addData("Intake parts found", intake.getPartsFound());
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            if (shooter.getTargetRpm() > 0) {
                telemetry.addData("Shooter", "%s  %.0f / %.0f RPM",
                        shooter.isAtSpeed() ? "READY" : "spinning up", shooter.getRpm(), shooter.getTargetRpm());
            } else {
                telemetry.addData("Shooter", "off (driver 2: right bumper)");
            }
        }
        telemetry.update();
    }
}
