package org.firstinspires.ftc.teamcode.detection;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class BaseCamera {
    boolean targetFound = false;
    List<AprilTagDetection> detections;
    int aprilTagId = -1;
    public TfodProcessor tfodProcessor;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    double pixelX = 0;
    double pixelY = 0;
    double pixelW = 0;
    double pixelH = 0;

    public BaseCamera(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        aprilTagProcessor.setDecimation(2);

        tfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .setModelAssetName("CenterStage.tflite")
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTagProcessor, tfodProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);
    }

    public void UsingCamera(Gamepad gamepad, Telemetry telemetry) {
        targetFound = false;
        detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                aprilTagId = detection.id;
                if (desiredTagId == detection.id) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    targetFound = false;
                }

            }
        }

        if (targetFound == false) {
            aprilTagId = -1;
        }

        if (gamepad.b) {
            visionPortal.stopLiveView();
        } else if (gamepad.x) {
            visionPortal.resumeLiveView();
        }

        telemetryTfod(telemetry);

        telemetry.addData("aprilTagId is ", "%d", aprilTagId);
        telemetry.addData("Founded", targetFound);
    }

    private void telemetryTfod(Telemetry telemetry) {

        List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            pixelX = (recognition.getLeft() + recognition.getRight()) / 2;
            pixelY = (recognition.getTop()  + recognition.getBottom()) / 2;

            pixelH = recognition.getHeight();
            pixelW = recognition.getWidth();
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", pixelX, pixelY);
            telemetry.addData("- Size", "%.0f x %.0f", pixelW, pixelH);
        }   // end for() loop
    }
}
