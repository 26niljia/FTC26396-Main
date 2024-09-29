package org.firstinspires.ftc.team26396.opmodes.test;

import static org.firstinspires.ftc.team26396.constants.Constants.LocalizerConstants.OPTICAL_ODOMETRY_NAME;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team26396.constants.ConstantsLoader;
import org.firstinspires.ftc.team26396.purepursuit.localization.OpticalLocalizer;
import org.firstinspires.ftc.team26396.utility.DataLogger;

@TeleOp(name = "Test - Kalman Filter", group = "Test")
public final class KalmanFilterTest extends OpMode {
    private OpticalLocalizer localizer;
    private DataLogger dataLogger;

    @Override public void init() {
        new ConstantsLoader().load();

        SparkFunOTOS opticalOdometry = hardwareMap.get(SparkFunOTOS.class, OPTICAL_ODOMETRY_NAME);
        dataLogger = new DataLogger("KalmanFilterTest");
        dataLogger.addLine("rx, ex, ry, ey, rh, eh");
        localizer = new OpticalLocalizer(opticalOdometry);
    }

    @Override public void loop() {
        localizer.update();

        SparkFunOTOS.Pose2D rawPose       = localizer.rawPose;
        SparkFunOTOS.Pose2D estimatedPose = localizer.pose;

        String line = rawPose.x + "," + estimatedPose.x + "," + rawPose.y + "," + estimatedPose.y
                + "," + rawPose.h + "," + estimatedPose.h;

        dataLogger.addLine(line);

        if (gamepad1.options || gamepad2.options) {
            dataLogger.save();
            dataLogger.clear();
        }

        localizer.debug(telemetry);
    }
}