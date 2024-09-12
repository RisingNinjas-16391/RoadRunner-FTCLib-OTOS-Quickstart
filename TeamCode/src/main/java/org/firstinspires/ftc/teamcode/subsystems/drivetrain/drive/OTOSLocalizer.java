package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOSLocalizer implements Localizer {
    Telemetry telemetry;
    SparkFunOTOS myOtos;
    com.arcrobotics.ftclib.geometry.Pose2d pose = new com.arcrobotics.ftclib.geometry.Pose2d();
    Pose2d poseVel = new Pose2d();

    com.arcrobotics.ftclib.geometry.Pose2d offset = new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, Rotation2d.fromDegrees(90));

    public OTOSLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        myOtos.setAngularUnit(AngleUnit.RADIANS);
        myOtos.setLinearUnit(DistanceUnit.INCH);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);


        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        myOtos.setPosition(RRPose2dToOTOSPose2d(pose2d));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVel;
    }

    @Override
    public void update() {
        pose = new com.arcrobotics.ftclib.geometry.Pose2d(new com.arcrobotics.ftclib.geometry.Pose2d(myOtos.getPosition().x, myOtos.getPosition().y, Rotation2d.fromDegrees(myOtos.getPosition().h)).getTranslation().rotateBy(offset.getRotation()).unaryMinus(), Rotation2d.fromDegrees(myOtos.getPosition().h));

        poseVel = OTOSPose2dToRRPose2d(myOtos.getVelocity());
    }

    private static Pose2d OTOSPose2dToRRPose2d(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }

    private static SparkFunOTOS.Pose2D RRPose2dToOTOSPose2d(Pose2d rrPose) {
        return new SparkFunOTOS.Pose2D(rrPose.getX(), rrPose.getY(), rrPose.getHeading());
    }
}
