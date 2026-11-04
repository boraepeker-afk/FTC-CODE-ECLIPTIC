package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/*
 * The tuners that show up on Pedro's AutoTune web page:
 * connect to the robot's Wi-Fi and open http://192.168.43.1:10158
 * Steps are in PedroConstants.
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new DriveEncoderLocalizer(hardwareMap),
                hardwareMap -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, PedroConstants.drivetrainConfig),
                hardwareMap -> new DriveEncoderLocalizer(hardwareMap),
                () -> new Foresight(PedroConstants.foresightConfig));
    }
}
