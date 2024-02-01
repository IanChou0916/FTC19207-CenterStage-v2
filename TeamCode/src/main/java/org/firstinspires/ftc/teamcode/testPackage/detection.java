package org.firstinspires.ftc.teamcode.testPackage;

import static org.firstinspires.ftc.teamcode.subsystem.intake.Intake.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveConstantstEST;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(name = "detection")
public class detection extends LinearOpMode {


    protected DistanceSensor sensorDistance;
    public int modNumber =0;

    public static double DISTANCE_RANGE = 10.0;
    public static double DEGREE_ERROR = 0.0;
    public static double dy = 32;
    //protected AutoField autoField;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // robotConfig to Hardware map.
        Intake intake = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // setPose and EstimatePosition.
        Pose2d startPose = new Pose2d(-36, 56, Math.toRadians(270));
        Pose2d detectPose = new Pose2d(-36,dy,Math.toRadians(270));
        Vector2d setPose = new Vector2d(-36,dy);
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        drive.setPoseEstimate(startPose);
        TrajectorySequence forward1 = drive.trajectorySequenceBuilder(new Pose2d(-36,dy,Math.toRadians(270)))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->  ArmDown())
                .UNSTABLE_addTemporalMarkerOffset(0.7,()-> ClawROpen())
                .waitSeconds(1.5)
                .build();
        TrajectorySequence forward2 = drive.trajectorySequenceBuilder(new Pose2d(-36,dy,Math.toRadians(0)))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->  ArmDown())
                .UNSTABLE_addTemporalMarkerOffset(0.7,()-> ClawROpen())
                .waitSeconds(1.5)
                .build();
        TrajectorySequence forward3 = drive.trajectorySequenceBuilder(new Pose2d(-36,dy,Math.toRadians(180)))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->  ArmDown())
                .UNSTABLE_addTemporalMarkerOffset(0.7,()-> ClawROpen())
                .waitSeconds(1.5)
                .build();

        TrajectorySequence detectionToLeft = drive.trajectorySequenceBuilder(new Pose2d(-36,dy,Math.toRadians(180)))
                .waitSeconds(1.0)
                .splineTo(
                        setPose,Math.toRadians(180+ DEGREE_ERROR),
                        SampleMecanumDrive.getVelocityConstraint(25,30,13.74),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .addDisplacementMarker(()->{
                    modNumber = 1;
                    Intake.ClawROpen();
                })
                .build();


        TrajectorySequence detectionTurnDegrees = drive.trajectorySequenceBuilder(detectPose)
                .splineTo(
                        setPose,Math.toRadians(0+ DEGREE_ERROR),
                        SampleMecanumDrive.getVelocityConstraint(25,30,DriveConstantstEST.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .addDisplacementMarker(()->{
                    if(sensorIsDetected()){
                        modNumber = 3;
                        ClawROpen(); // open purple pixel.
                    }
                    else {
                        drive.followTrajectorySequence(detectionToLeft);
                    }
                })

                .build();

        TrajectorySequence open = drive.trajectorySequenceBuilder(startPose) // run to detect modNumber.
                .splineTo(
                        setPose,Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(25,30,13.74),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstantstEST.MAX_ACCEL)
                )
                .addTemporalMarker(1.75, () -> {
                    // This marker runs two seconds into the trajectory
                    // Run your action in here!
                })
                .addDisplacementMarker(() -> {
                    if(sensorIsDetected()) {
                        modNumber = 2;
                        drive.followTrajectorySequence(forward1);
                        // open purple pixel.

                    }
                    else {
                        drive.followTrajectorySequence(detectionTurnDegrees);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() ->  ClawROpen())
                .UNSTABLE_addTemporalMarkerOffset(0.7,()-> ArmUp())
                .waitSeconds(1.5)

                .lineToLinearHeading(detectPose)
                .build();
        waitForStart();

        drive.followTrajectorySequence(open);
        telemetryForConfig();
    }



    protected double getDistanceValue(){
        return sensorDistance.getDistance(DistanceUnit.CM);
    }
    protected boolean sensorIsDetected(){
        return getDistanceValue()<DISTANCE_RANGE;
    }

    public int getModNumber(){
        return modNumber;
    }

    public void telemetryForConfig(){
        telemetry.addData("ANGLE_ERROR","%.2f", DEGREE_ERROR);
        telemetry.addData("DISTANCE_RANGE",DISTANCE_RANGE);
        telemetry.addData("dy",dy);
        telemetry.addData("modNumber",getModNumber());
        telemetry.update();

    }
    //protected void






}
