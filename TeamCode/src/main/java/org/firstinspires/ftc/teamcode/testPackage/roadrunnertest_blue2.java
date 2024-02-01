package org.firstinspires.ftc.teamcode.testPackage;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Autonomous(name = "blueFront",group = "Auto")
public class roadrunnertest_blue2 extends LinearOpMode {


    public static double MAX_VEL = 62.41224921811162;
    public static double MAX_ACCEL = 52.48291908330528;
    public static double MAX_ANG_VEL = 2.8596465587615967;
    public static double MAX_ANG_ACCEL = Math.toRadians(240.5639808);

    Trajectory traj1,traj2,traj3;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException{
        Intake take = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 56, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(45,35,Math.toRadians(270)))
                //.splineTo(new Vector2d(12,34),Math.toRadians(270))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(20)
                //.back(29)
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .back(18)
                //.strafeRight(20)
                .build();

        waitForStart();

        if(isStopRequested());


            drive.followTrajectory(traj1);
            drive.turn(Math.toRadians(-96));
            take.ClawtrunDown();
            sleep(500);
            take.ArmUp();
            take.SlideUp();
            sleep(1500);
            take.ClawROpen();
            sleep(500);
            take.ClawRClose();
            sleep(500);
            take.SlideDown();
            take.ArmDown();
            sleep(200);
            take.SlideUp2();
            sleep(1000);
            take.ClawLOpen();
            sleep(500);
            take.SlideDown();
            sleep(1500);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);


    }

}

