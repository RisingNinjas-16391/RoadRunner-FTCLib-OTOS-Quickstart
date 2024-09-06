package org.firstinspires.ftc.teamcode.subsystem.drivetrain.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DrivetrainSubsystem;

/**
 * This is a simple routine to test translational drive capabilities.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends CommandOpMode {

    public static double DISTANCE = 60; // in

    private DrivetrainSubsystem drive;
    private FollowTrajectoryCommand strafeFollower;

    @Override
    public void initialize() {
        drive = new DrivetrainSubsystem(hardwareMap, false);
        strafeFollower = new FollowTrajectoryCommand(drive,
                drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(DISTANCE)
                    .build()
        );
        schedule(new WaitUntilCommand(this::isStarted).andThen(strafeFollower.whenFinished(() -> {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        })));
    }

}
