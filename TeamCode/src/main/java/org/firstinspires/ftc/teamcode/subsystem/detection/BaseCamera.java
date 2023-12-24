package org.firstinspires.ftc.teamcode.CusVisual;

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
import org.firstinspires.ftc.teamcode.CusDrive.BaseChassis;
import org.firstinspires.ftc.teamcode.RobotContant;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class BaseCamera {
    static RobotContant robotContant = new RobotContant();
    DcMotorEx LF = null;
    DcMotorEx RF = null;
    DcMotorEx LB = null;
    DcMotorEx RB = null;
    double DESIRED_DISTANCE = robotContant.Robot_DESIRED_DISTANCE;
    AprilTagDetection desiredTag = null;
    public static int desiredTagId = robotContant.RobotDesiredTagId;
    FtcDashboard dashboard;
    boolean targetFound = false;
    final double X_GAIN  =  robotContant.X_GAIN;
    final double Y_GAIN =  robotContant.Y_GAIN;
    final double TURN_GAIN   = robotContant.TURN_GAIN;
    final double MAX_AUTO_SPEED = robotContant.MAX_AUTO_X;
    final double MAX_AUTO_STRAFE= robotContant.MAX_AUTO_Y;
    final double MAX_AUTO_TURN  = robotContant.MAX_AUTO_TURN;

    List<AprilTagDetection> detections;
    int myAprilTagIdCode = 0;
    TfodProcessor tfodProcessor;
    AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    double pixelX = 0;
    double pixelY = 0;
    double pixelW = 0;
    double pixelH = 0;

    public BaseCamera(HardwareMap hardwareMap, boolean drive) {
        dashboard = FtcDashboard.getInstance();

        if (drive) {
            BaseChassis baseChassis = new BaseChassis(hardwareMap, true);
            this.LF = baseChassis.LF;
            this.LB = baseChassis.LB;
            this.RF = baseChassis.RF;
            this.RB = baseChassis.RB;
        }

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
                myAprilTagIdCode = detection.id;
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
            myAprilTagIdCode = -1;
        }

        if (gamepad.b) {
            visionPortal.stopLiveView();
        } else if (gamepad.x) {
            visionPortal.resumeLiveView();
        }

        telemetryTfod(telemetry);

        telemetry.addData("aprilTagId is ", "%d", myAprilTagIdCode);
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

    public void DriveCamera(Gamepad gamepad) throws InterruptedException {
        double xPow = 0;
        double yPow = 0;
        double turnPow = 0;

        if (targetFound && gamepad.a) {
            double YError = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
            double XError = desiredTag.ftcPose.x;
            double yawError = desiredTag.ftcPose.yaw;

            xPow = Range.clip(YError * X_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turnPow = Range.clip(XError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yPow = Range.clip(-yawError * Y_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


            sleep(10);
        } else {
            xPow = 0;
            yPow = 0;
            turnPow = 0;
        }
        moveRobot(yPow, xPow, turnPow);
    }

    public void moveRobot(double y, double x, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  y -x -yaw;
        double rightFrontPower   =  y +x +yaw;
        double leftBackPower     =  y +x -yaw;
        double rightBackPower    =  y -x +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        LF.setPower(leftFrontPower);
        RF.setPower(rightFrontPower);
        LB.setPower(leftBackPower);
        RB.setPower(rightBackPower);
    }
}
