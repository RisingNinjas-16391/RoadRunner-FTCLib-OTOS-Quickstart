package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequenceCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private final TrajectorySequence trajectory;

    public FollowTrajectorySequenceCommand(DrivetrainSubsystem drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
