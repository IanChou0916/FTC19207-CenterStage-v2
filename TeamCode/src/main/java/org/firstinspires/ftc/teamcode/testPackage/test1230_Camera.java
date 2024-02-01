package org.firstinspires.ftc.teamcode.testPackage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.detection.BaseCamera;


public class test1230_Camera extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BaseCamera camera = new BaseCamera(hardwareMap, "red");

        waitForStart();
        while (opModeIsActive()) {
            camera.UsingCamera(gamepad1, telemetry);
        }

    }
}
