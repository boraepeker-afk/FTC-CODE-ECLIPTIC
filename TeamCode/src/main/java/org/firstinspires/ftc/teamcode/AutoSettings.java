package org.firstinspires.ftc.teamcode;

/*
 * The choices drivers make on the Driver Hub during Init, before autonomous starts.
 */
public class AutoSettings {

    public enum Alliance { BLUE, RED }

    public Alliance alliance = Alliance.BLUE;
    public int startDelaySeconds = 0;   // wait this long before moving (to stay out of a partner's way)
    public boolean collectMore = true;  // after shooting the preloads, go pick up more POLLEN and shoot again

    public boolean isRed() {
        return alliance == Alliance.RED;
    }
}
