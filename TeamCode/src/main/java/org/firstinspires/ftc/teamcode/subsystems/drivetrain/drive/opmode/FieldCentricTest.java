package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This is an additional test made specifically for FTCLib's quickstart.
 * It tests the field-centric mode of the {@link DrivetrainSubsystem}.
 * The robot should always drive in the direction in which the left stick is
 * being pushed. This should be run after your {@link LocalizationTest}
 * to ensure proper tuning for the localization's heading estimate.
 *
 * @author Jackson
 */
@Config
@TeleOp
public class FieldCentricTest extends CommandOpMode {

    private GamepadEx gamepad;
    private DrivetrainSubsystem drive;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        drive = new DrivetrainSubsystem(hardwareMap, telemetry, true);

        register(drive);
        drive.setDefaultCommand(new MecanumDriveCommand(
                drive, () -> -gamepad.getLeftY(), gamepad::getLeftX, gamepad::getRightX
        ));

        schedule(new RunCommand(() -> {
            drive.update();
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }));
    }

}
