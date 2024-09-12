package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name = "BlueAuto", group = "Autonomous")
public class BlueAuto extends CommandOpMode {
    private Telemetry m_telemetry;
    @Override
    public void initialize() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 1); //Uses heavily modified untested hardware

        waitForStart();

    }
}