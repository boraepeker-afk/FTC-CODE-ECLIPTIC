package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Pedro Pathing settings (Pedro 3). Called PedroConstants so it isn't mixed up with our main Constants.
 *
 * HOW TO TUNE (with the robot, connected to its Wi-Fi):
 *   1. Build and install the code.
 *   2. Open http://192.168.43.1:10158 in a browser (this is Pedro's "AutoTune" page).
 *   3. "Mecanum" procedure: checks the drive motor names and directions. Copy the
 *      directions it gives you into the four *_DIRECTION lines below if they differ.
 *   4. "Tests" -> "Localization Test": drive around; forward must make x go UP and
 *      strafing left must make y go UP. If not, fix the directions / DriveConstants first.
 *   5. "Foresight" procedure: the robot drives itself to measure its speeds and braking.
 *      It needs about 5 feet of clear space in front and to the left of the robot.
 *      Copy the numbers from its Java tab into foresightConfig below, then set FORESIGHT_TUNED = true.
 *   6. "Tests" -> "Line Test" / "Curve Test" to check it follows paths.
 */
public class PedroConstants {

    // Set to true only AFTER pasting real numbers from the Foresight tuner into foresightConfig.
    // While false, the Pedro autonomous will not drive (the numbers below belong to someone else's robot).
    public static final boolean FORESIGHT_TUNED = false;

    // ---- Drive motors: same names and directions as DriveSubsystem ----
    public static final String FRONT_LEFT = "front_left";
    public static final String BACK_LEFT = "back_left";
    public static final String FRONT_RIGHT = "front_right";
    public static final String BACK_RIGHT = "back_right";
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(FRONT_LEFT);
                c.backLeftName.set(BACK_LEFT);
                c.frontRightName.set(FRONT_RIGHT);
                c.backRightName.set(BACK_RIGHT);

                c.frontLeftDirection.set(FRONT_LEFT_DIRECTION);
                c.backLeftDirection.set(BACK_LEFT_DIRECTION);
                c.frontRightDirection.set(FRONT_RIGHT_DIRECTION);
                c.backRightDirection.set(BACK_RIGHT_DIRECTION);

                c.manualBrakeMode.set(true);
            }
    );

    // ---- Foresight (the path-following brain) ----
    // PLACEHOLDER NUMBERS from Pedro's documentation example robot. Replace ALL of this block with
    // what the Foresight tuner's Java tab gives you (step 5 above).
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));

                c.headingFeedback.set(Controller.proportional(5.258721785960744));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
            }
    );

    /** Makes the Follower: our drive-encoder tracker + the mecanum drivetrain + Foresight. */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new DriveEncoderLocalizer(hardwareMap),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
