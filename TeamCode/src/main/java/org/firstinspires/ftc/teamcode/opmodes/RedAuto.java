package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name = "RedAuto", group = "Autonomous")
public class RedAuto extends CommandOpMode {
    private RobotContainer m_robotContainer;
    private Telemetry m_telemetry;
    @Override
    public void initialize() {
        m_robotContainer = new RobotContainer(hardwareMap, gamepad1, gamepad2, 2); //Uses heavily modified untested hardware

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

    }
    @Override
    public void run() {
        super.run();
        m_robotContainer.periodic(m_telemetry);

    }

}