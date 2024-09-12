package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class OTOSLimelightLocalizer implements Localizer {
    private Telemetry m_telemetry;
    private final SparkFunOTOS m_otos;
    private final Limelight3A m_limelight;


    private Pose2d m_pose;
    private Pose2d m_otosOffset;

    private ElapsedTime timer = new ElapsedTime();

    public OTOSLimelightLocalizer(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose) {
        m_telemetry = telemetry;

        m_otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        m_limelight = hardwareMap.get(Limelight3A.class, "limelight");

        m_pose = startPose;

        m_otosOffset = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

        configureOtos();
        configureLimelight();

        timer.startTime();
    }

    private void configureOtos() {
        m_telemetry.addLine("Configuring OTOS...");

        m_otos.setLinearUnit(DistanceUnit.INCH);
        m_otos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(m_otosOffset.getX(), m_otosOffset.getY(), m_otosOffset.getRotation().getDegrees());
        m_otos.setOffset(offset);

        m_otos.setLinearScalar(1.0);
        m_otos.setAngularScalar(1.0);


        m_otos.calibrateImu();

        m_otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        m_otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        m_otos.getVersionInfo(hwVersion, fwVersion);

        m_telemetry.addLine("OTOS configured! Press start to get position data!");
        m_telemetry.addLine();
        m_telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        m_telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));

    }

    private void configureLimelight() {
        m_telemetry.setMsTransmissionInterval(11);

        m_limelight.pipelineSwitch(0);

        m_limelight.start();
    }

    @Override
    public void update() {
        double loopTime = timer.time();
        SparkFunOTOS.Pose2D otosVel = m_otos.getVelocity();

        Pose2d vel = new Pose2d(otosVel.x, otosVel.y, Rotation2d.fromDegrees(otosVel.h));
        Twist2d twist = new Twist2d(vel.getX() * loopTime, vel.getY() * loopTime, vel.getRotation().getRadians() * loopTime);

        m_pose = m_pose.exp(twist);

        Pose2d cameraPose = getLLData();

        Twist2d robotToCamera = m_pose.log(cameraPose);

        m_pose = m_pose.exp(new Twist2d(robotToCamera.dx * 0.05, robotToCamera.dy * 0.05, robotToCamera.dtheta * 0.05));

        timer.reset();
    }

    private Pose2d getLLData() {
        LLStatus status = m_limelight.getStatus();
        m_telemetry.addData("Name", "%s",
                status.getName());
        m_telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        m_telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = m_limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            m_telemetry.addData("LL Latency", captureLatency + targetingLatency);
            m_telemetry.addData("Parse Latency", parseLatency);
            m_telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                m_telemetry.addData("tx", result.getTx());
                m_telemetry.addData("txnc", result.getTxNC());
                m_telemetry.addData("ty", result.getTy());
                m_telemetry.addData("tync", result.getTyNC());

                m_telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    m_telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    m_telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    m_telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    m_telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    m_telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
            return new Pose2d(botpose.getPosition().x, botpose.getPosition().y, Rotation2d.fromDegrees(botpose.getOrientation().getYaw()));
        } else {
            m_telemetry.addData("Limelight", "No data available");
            return null;
        }
    }
    @NonNull
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(m_pose.getX(), m_pose.getY(), m_pose.getRotation().getRadians());
    }

    @Nullable
    @Override
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NonNull com.acmerobotics.roadrunner.geometry.Pose2d pose2d) {
        m_pose = new Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading()));
    }
}
