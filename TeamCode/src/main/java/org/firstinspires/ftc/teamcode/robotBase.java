package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.slider.slider;

@Config

public abstract class robotBase extends LinearOpMode {
    protected SampleMecanumDrive drive;
    protected slider sliders;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        sliders = new slider(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        robotInit();

        robotStart();
    }
    protected abstract void robotInit();

    protected abstract void robotStart() throws InterruptedException;
}