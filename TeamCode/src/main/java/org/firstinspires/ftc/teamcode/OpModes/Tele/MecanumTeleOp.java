package org.firstinspires.ftc.teamcode.OpModes.Tele;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * The TeleOp. All the setup is in Bot and BotContainer; this file just starts them.
 */
@TeleOp(name = "Mecanum TeleOp")
public class MecanumTeleOp extends CommandOpMode {

    private Bot bot;

    // Runs once when you press Init.
    @Override
    public void initialize() {
        reset();                // clear anything left over from the last OpMode
        Constants.init(this);   // connect hardwareMap, telemetry and the gamepads
        bot = new Bot(Bot.OpMode.TELEOP);
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
