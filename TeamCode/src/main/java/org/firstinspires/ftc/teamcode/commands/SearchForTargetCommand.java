package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/*
 * Turns the robot slowly in place until the camera sees an AprilTag target.
 * Finishes as soon as a target is visible. Give it a timeout (withTimeout) so it
 * doesn't spin forever if there's nothing to find.
 */
public class SearchForTargetCommand extends CommandBase {

    // Turning power while searching (set in Constants.AutoConstants).
    private static final double SEARCH_TURN = Constants.AutoConstants.SEARCH_TURN;

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    public SearchForTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(0, 0, SEARCH_TURN, 1.0);
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
