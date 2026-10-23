package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * The autonomous: find the AprilTag target and drive up to it.
 * The steps are in BotContainer.getAutonomousCommand(); the numbers are in Constants.AutoConstants.
 */
@Autonomous(name = "AprilTag Auto")
public class AprilTagAuto extends CommandOpMode {

    private Bot bot;

    // Runs once when you press Init.
    @Override
    public void initialize() {
        reset();                // clear anything left over from the last OpMode
        Constants.init(this);   // connect hardwareMap, telemetry and the gamepads
        bot = new Bot(Bot.OpMode.AUTO);
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
