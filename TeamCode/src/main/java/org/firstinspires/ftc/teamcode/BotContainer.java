package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.commands.AimAtTargetCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToTargetCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
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
    private final ElapsedTime autoTimer = new ElapsedTime();

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
     * AUTONOMOUS (30 seconds). The spots it drives to are in Constants.AutoConstants.
     *   1. Wait for the start delay (if any), turn the flywheel on.
     *   2. Drive to the shooting spot with the encoders.                   -> LEAVE (3 pts)
     *   3. If the camera sees the target, fine-aim with it. Wait for the flywheel,
     *      then run the intake to shoot the 4 preloaded POLLEN.             -> HIVE TIP (20 pts)
     *   4. If there's time (and "collect more" is on): drive through the row of POLLEN with the
     *      intake running, come back to the shooting spot, aim and shoot again.
     *   5. Drive to the Loading Zone and stop.                              -> PARK (5 pts)
     * Every step has a time limit, so one problem can't use up the whole 30 seconds.
     */
    public Command getAutonomousCommand(AutoSettings settings) {
        boolean mirror = settings.isRed() && AutoConstants.MIRROR_FOR_RED;
        double[] shootPose        = mirrored(AutoConstants.SHOOT_POSE, mirror);
        double[] collectStartPose = mirrored(AutoConstants.COLLECT_START_POSE, mirror);
        double[] collectEndPose   = mirrored(AutoConstants.COLLECT_END_POSE, mirror);
        double[] parkPose         = mirrored(AutoConstants.PARK_POSE, mirror);

        Command collectAndShootAgain = new SequentialCommandGroup(
                step("4. going to the POLLEN"),
                new DriveToPoseCommand(drive, collectStartPose, AutoConstants.DRIVE_SPEED)
                        .withTimeout(AutoConstants.DRIVE_TIMEOUT_MS),
                new InstantCommand(() -> intake.setPower(AutoConstants.INTAKE_POWER), intake),
                step("4. collecting"),
                new DriveToPoseCommand(drive, collectEndPose, AutoConstants.COLLECT_SPEED)
                        .withTimeout(AutoConstants.COLLECT_TIMEOUT_MS),
                new InstantCommand(intake::stop, intake),
                step("4. back to the shooting spot"),
                new DriveToPoseCommand(drive, shootPose, AutoConstants.DRIVE_SPEED)
                        .withTimeout(AutoConstants.DRIVE_TIMEOUT_MS),
                // Only shoot again if there's still time before parking.
                new ConditionalCommand(
                        shootSequence("4"),
                        new InstantCommand(),
                        () -> autoTimer.seconds() < AutoConstants.PARK_BY_S));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    autoTimer.reset();
                    drive.resetPose();
                }),
                step("1. start delay"),
                new WaitCommand(settings.startDelaySeconds * 1000L),

                // Flywheel on now, so it's at speed by the time the robot gets to the shooting spot.
                new InstantCommand(() -> shooter.setTargetRpm(AutoConstants.AUTO_SHOOT_RPM), shooter),
                step("2. driving to the shooting spot"),
                new DriveToPoseCommand(drive, shootPose, AutoConstants.DRIVE_SPEED)
                        .withTimeout(AutoConstants.DRIVE_TIMEOUT_MS),

                shootSequence("3"),

                new ConditionalCommand(
                        collectAndShootAgain,
                        new InstantCommand(),
                        () -> settings.collectMore && autoTimer.seconds() < AutoConstants.COLLECT_CUTOFF_S),

                new InstantCommand(shooter::stop, shooter),
                new InstantCommand(intake::stop, intake),
                step("5. parking"),
                new DriveToPoseCommand(drive, parkPose, AutoConstants.DRIVE_SPEED)
                        .withTimeout(AutoConstants.DRIVE_TIMEOUT_MS),
                new InstantCommand(drive::stop, drive),
                step("done"));
    }

    // Aim (with the camera if it sees the target), wait for the flywheel, then feed the balls in.
    private Command shootSequence(String number) {
        return new SequentialCommandGroup(
                step(number + ". aiming"),
                new ConditionalCommand(
                        new DriveToTargetCommand(drive, vision).withTimeout(AutoConstants.AIM_TIMEOUT_MS),
                        new InstantCommand(),
                        () -> AutoConstants.USE_CAMERA_AIM && vision.hasTarget()),
                new InstantCommand(drive::stop, drive),
                step(number + ". waiting for the flywheel"),
                new WaitUntilCommand(shooter::isAtSpeed).withTimeout(AutoConstants.SPIN_UP_TIMEOUT_MS),
                step(number + ". shooting"),
                new RunCommand(() -> intake.setPower(AutoConstants.FEED_POWER), intake)
                        .withTimeout(AutoConstants.FEED_MS),
                new InstantCommand(intake::stop, intake));
    }

    private Command step(String name) {
        return new InstantCommand(() -> autoStep = name);
    }

    // For RED: flip left/right (y) and the turn direction (heading).
    private static double[] mirrored(double[] pose, boolean mirror) {
        return mirror ? new double[]{pose[0], -pose[1], -pose[2]} : pose;
    }

    // ---- Driver Hub screen ----

    public void showInitTelemetry(Bot.OpMode opMode) {
        Telemetry telemetry = Constants.telemetry;
        telemetry.addData("Camera", vision.isCameraConnected()
                ? "found" : "NOT found - name it \"" + Constants.VisionConstants.CAMERA_NAME + "\" in the configuration");
        if (opMode == Bot.OpMode.TELEOP) {
            telemetry.addData("Intake parts found", intake.getPartsFound());
            telemetry.addData("Shooter", shooter.getStatus());
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
            telemetry.addData("Time", "%.1f s", autoTimer.seconds());
            telemetry.addData("Position", "x %.1f in, y %.1f in, heading %.0f deg",
                    drive.getX(), drive.getY(), drive.getPoseHeadingDegrees());
            telemetry.addData("Shooter", shooter.getStatus());
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
            telemetry.addData("Shooter", shooter.getStatus());
        }
        telemetry.update();
    }
}
