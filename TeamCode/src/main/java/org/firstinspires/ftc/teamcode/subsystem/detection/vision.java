package org.firstinspires.ftc.teamcode.subsystem.detection;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class vision {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor myAprilTagProcessor;
    private TfodProcessor myTfodProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myvisionPortal;
    public vision(HardwareMap hardwareMap){
        initAprilTag();
        initTfod();
        myvisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }
    public void initAprilTag() {
// Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    }
    public void initTfod() {
        myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
    }



}
