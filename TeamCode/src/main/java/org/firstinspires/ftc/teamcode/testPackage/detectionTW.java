package org.firstinspires.ftc.teamcode.testPackage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotAutoBase;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveConstantstEST;
import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Config
@Disabled
@Autonomous(name = "BlueBack.")
public class detectionTW extends robotAutoBase {
    protected AutoField autoField = AutoField.BLUE_BACK;


    @Override
    protected void robotInit() {
        // setPose and EstimatePosition.

        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");

        Pose2d startPose = estimatedPose.get(autoField);
        Pose2d detectPose = new Pose2d(-36,dy,Math.toRadians(270));
        Vector2d setPose = new Vector2d(-36,dy);
        drive.setPoseEstimate(startPose);


        open = drive.trajectorySequenceBuilder(startPose) // run to detect modNumber.
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
                        intake.ClawROpen(); // open purple pixel.
                    }
                    else {
                        drive.followTrajectorySequence(detectionTurnDegrees);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> Intake.ClawRClose())
                .UNSTABLE_addTemporalMarkerOffset(0.7,()-> intake.ArmUp())
                .waitSeconds(1.5)

                .lineToLinearHeading(detectPose)
                .build();
        detectionTurnDegrees = drive.trajectorySequenceBuilder(detectPose)
                .splineTo(
                        setPose,Math.toRadians(0+ DEGREE_ERROR),
                        SampleMecanumDrive.getVelocityConstraint(25,30,DriveConstantstEST.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .addDisplacementMarker(()->{
                    if(sensorIsDetected()){
                        modNumber = 3;
                        intake.ClawROpen(); // open purple pixel.
                    }
                    else {
                        drive.followTrajectorySequence(detectionToLeft);
                    }
                })

                .build();
        detectionToLeft = drive.trajectorySequenceBuilder(new Pose2d(-36,dy,Math.toRadians(180)))
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
        waitForStart();

    }

    @Override
    protected void robotStart() throws InterruptedException {
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





}
