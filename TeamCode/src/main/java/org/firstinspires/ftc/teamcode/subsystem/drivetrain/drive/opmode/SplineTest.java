package org.firstinspires.ftc.teamcode.subsystem.drivetrain.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DrivetrainSubsystem;

/**
 * This is an example of a more complex path to really test the tuning.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Autonomous(group = "drive")
public class SplineTest extends CommandOpMode {

    private DrivetrainSubsystem drive;
    private FollowTrajectoryCommand splineFollower;

    @Override
    public void initialize() {
        drive = new DrivetrainSubsystem(hardwareMap, false);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        splineFollower = new FollowTrajectoryCommand(drive, traj);
        schedule(new WaitUntilCommand(this::isStarted).andThen(
                splineFollower.andThen(new WaitCommand(2000),
                new FollowTrajectoryCommand(drive,
                    drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
                ))
        ));
    }

}
