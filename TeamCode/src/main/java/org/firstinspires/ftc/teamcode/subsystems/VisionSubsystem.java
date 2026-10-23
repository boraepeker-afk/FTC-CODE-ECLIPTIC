package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * The camera subsystem: looks for AprilTags every loop and remembers the best target.
 *
 * This season (BIOBUZZ) the tags come in "clusters" (groups of tags), and the SDK puts
 * each cluster's aiming point in the center of the Cell opening. So clusters are preferred
 * as targets; single tags are used only when no cluster is visible.
 *
 * The camera must be named "Webcam 1" in your Driver Station configuration.
 * If it isn't found, this subsystem just reports "no target" and the robot still drives.
 * Get it with VisionSubsystem.getInstance().
 */
public class VisionSubsystem extends SubsystemBase {

    // ---- One shared copy of this subsystem (same idea as getInstance() in the Astrobots code) ----
    private static VisionSubsystem instance = null;

    /** Gets the robot's one VisionSubsystem, creating it the first time. */
    public static synchronized VisionSubsystem getInstance() {
        if (instance == null) {
            instance = new VisionSubsystem();
        }
        return instance;
    }

    /** Forgets the old copy. Bot calls this at Init so every OpMode starts with fresh hardware. */
    public static synchronized void clearInstance() {
        instance = null;
    }
    // ---------------------------------------------------------------------------------------------

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // What the camera saw on the latest loop.
    private int detectionCount = 0;
    private boolean hasTarget = false;
    private String targetName = "none";
    private double targetRange = 0;    // inches from the camera to the target
    private double targetBearing = 0;  // degrees to turn to face it (positive = target is to the left)
    private double targetYaw = 0;      // degrees the target is rotated relative to the camera

    private VisionSubsystem() {
        HardwareMap hardwareMap = Constants.hardwareMap;
        // If there's no camera in the configuration, keep going without one so the robot can still drive.
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, Constants.VisionConstants.CAMERA_NAME);
        if (camera == null) {
            aprilTag = null;
            visionPortal = null;
            return;
        }

        // Uses this season's tag library by default.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(Constants.VisionConstants.DECIMATION);

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTag)
                .build();
    }

    /** False if no camera named "Webcam 1" was found in the configuration. */
    public boolean isCameraConnected() {
        return visionPortal != null;
    }

    // Runs automatically every loop, before the commands run.
    @Override
    public void periodic() {
        if (aprilTag == null) {
            return;  // no camera
        }
        List<AprilTagDetection> detections = aprilTag.getDetections();
        detectionCount = detections.size();

        AprilTagDetection best = null;
        String bestName = "none";
        boolean bestIsCluster = false;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                // Prefer clusters; among clusters, prefer the closest one.
                if (!bestIsCluster || cluster.ftcPose.range < best.ftcPose.range) {
                    best = cluster;
                    bestName = cluster.metadata.name;
                    bestIsCluster = true;
                }
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                // Tags missing from the library have no distance info, so skip them.
                if (single.metadata == null || bestIsCluster) {
                    continue;
                }
                if (best == null || single.ftcPose.range < best.ftcPose.range) {
                    best = single;
                    bestName = "Tag " + single.id + " (" + single.metadata.name + ")";
                }
            }
        }

        hasTarget = best != null;
        if (hasTarget) {
            targetName = bestName;
            targetRange = best.ftcPose.range;
            targetBearing = best.ftcPose.bearing;
            targetYaw = best.ftcPose.yaw;
        } else {
            targetName = "none";
        }
    }

    public boolean hasTarget()        { return hasTarget; }
    public String getTargetName()     { return targetName; }
    public double getTargetRange()    { return targetRange; }
    public double getTargetBearing()  { return targetBearing; }
    public double getTargetYaw()      { return targetYaw; }
    public int getDetectionCount()    { return detectionCount; }

    /** Turns the camera off. Call this when the OpMode ends. */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
