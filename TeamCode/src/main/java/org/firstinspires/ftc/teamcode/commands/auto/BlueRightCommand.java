package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.DrivetrainSubsystem;

public class BlueRightCommand extends SequentialCommandGroup {
    public BlueRightCommand(DrivetrainSubsystem drive) {
        addCommands(
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .forward(10)
                        .build()
                )
        );
    }
}