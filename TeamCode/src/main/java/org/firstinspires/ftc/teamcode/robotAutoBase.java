package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.testPackage.AutoField;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

@Config
@Disabled
public abstract class robotAutoBase extends LinearOpMode {
    protected TrajectorySequence open;
    protected TrajectorySequence detectionTurnDegrees;
    protected TrajectorySequence detectionToLeft;
    protected DistanceSensor sensorDistance;
    protected int modNumber =0;
    protected Intake intake;
    protected SampleMecanumDrive drive;
    public static double DISTANCE_RANGE = 20.0;
    public static double DEGREE_ERROR = 0.0;
    public static double dy = 32;
    public static HashMap<AutoField,Pose2d> estimatedPose = new HashMap<AutoField,Pose2d>(){{
       put(AutoField.BLUE_BACK,new Pose2d(-36, 56, Math.toRadians(270)));
       put(AutoField.BLUE_FRONT,new Pose2d(12, 56, Math.toRadians(270)));
       put(AutoField.RED_FRONT,new Pose2d(12, -58, Math.toRadians(90)));
       put(AutoField.RED_BACK,new Pose2d(-36, -58, Math.toRadians(90)));
    }};


    //public static final double DISTANCE_RANGE = 20.0;
    //public static final double DEGREE_EEROR = 0.0;


    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, 56, Math.toRadians(270));
        Pose2d detectPose = new Pose2d(-36,10,Math.toRadians(270));
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        drive.setPoseEstimate(startPose);

        telemetry.setMsTransmissionInterval(50);

        robotInit();

        robotStart();


    }
    protected abstract void robotInit();

    protected abstract void robotStart() throws InterruptedException;
}