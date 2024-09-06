package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOSLocalizer implements Localizer {
    SparkFunOTOS myOtos;
    Pose2d pose = new Pose2d();
    Pose2d poseVel = new Pose2d();

    public OTOSLocalizer(HardwareMap hardwareMap) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        myOtos.setAngularUnit(AngleUnit.RADIANS);
        myOtos.setLinearUnit(DistanceUnit.INCH);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 90);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        myOtos.calibrateImu();

        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 90);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return pose;
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
        pose = OTOSPose2dToRRPose2d(myOtos.getPosition());
        poseVel = OTOSPose2dToRRPose2d(myOtos.getVelocity());
    }

    private static Pose2d OTOSPose2dToRRPose2d(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }

    private static SparkFunOTOS.Pose2D RRPose2dToOTOSPose2d(Pose2d rrPose) {
        return new SparkFunOTOS.Pose2D(rrPose.getX(), rrPose.getY(), rrPose.getHeading());
    }
}
