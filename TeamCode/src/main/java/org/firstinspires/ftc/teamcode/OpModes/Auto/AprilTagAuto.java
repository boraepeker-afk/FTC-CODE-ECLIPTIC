package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoSettings;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * The autonomous. The steps are in BotContainer.getAutonomousCommand();
 * the spots and numbers are in Constants.AutoConstants.
 *
 * During Init, driver 1 picks the settings on gamepad 1:
 *   X (blue button) = BLUE alliance      B (red button) = RED alliance
 *   D-pad up/down   = start delay +1 / -1 second
 *   Y               = collect more POLLEN on / off
 * Then press Start.
 */
@Autonomous(name = "AprilTag Auto")
public class AprilTagAuto extends CommandOpMode {

    private Bot bot;

    // Runs when you press Init, and keeps running the menu until you press Start.
    @Override
    public void initialize() {
        reset();                // clear anything left over from the last OpMode
        Constants.init(this);   // connect hardwareMap, telemetry and the gamepads
        bot = new Bot(Bot.OpMode.AUTO);

        AutoSettings settings = new AutoSettings();
        boolean upWasPressed = false, downWasPressed = false, yWasPressed = false;

        while (opModeInInit()) {
            if (gamepad1.x) settings.alliance = AutoSettings.Alliance.BLUE;
            if (gamepad1.b) settings.alliance = AutoSettings.Alliance.RED;

            // Only count a button once per press, not every loop while it's held.
            if (gamepad1.dpad_up && !upWasPressed) {
                settings.startDelaySeconds = Math.min(15, settings.startDelaySeconds + 1);
            }
            if (gamepad1.dpad_down && !downWasPressed) {
                settings.startDelaySeconds = Math.max(0, settings.startDelaySeconds - 1);
            }
            if (gamepad1.y && !yWasPressed) {
                settings.collectMore = !settings.collectMore;
            }
            upWasPressed = gamepad1.dpad_up;
            downWasPressed = gamepad1.dpad_down;
            yWasPressed = gamepad1.y;

            super.run();  // keeps the camera and position tracking updating, so you can check them

            telemetry.addData("ALLIANCE", "%s   (X = blue, B = red)", settings.alliance);
            telemetry.addData("Start delay", "%d s   (d-pad up / down)", settings.startDelaySeconds);
            telemetry.addData("Collect more", "%s   (Y to change)", settings.collectMore ? "YES" : "no");
            telemetry.addLine();
            bot.updateTelemetry();  // also calls telemetry.update()
            sleep(20);
        }

        // Start was pressed: set up the routine with these choices. It starts running right away.
        bot.startAuto(settings);
    }

    // Runs over and over after you press Start.
    @Override
    public void run() {
        super.run();            // runs the subsystems and commands
        bot.updateTelemetry();
    }

    // Runs the OpMode, then always turns the camera off at the end.
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
