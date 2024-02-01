package org.firstinspires.ftc.teamcode.testPackage;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Autonomous(name = "redFront",group = "Auto")
public class roadrunnertest_red2 extends LinearOpMode {


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
        Pose2d startPose = new Pose2d(12, -58, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12,-30),Math.toRadians(90))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .back(35)
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(30)
                .build();

        waitForStart();

        if(isStopRequested());


        drive.followTrajectory(traj1);
        take.ClawtrunDown();
        sleep(500);
        drive.turn(Math.toRadians(100));
        drive.followTrajectory(traj2);
        take.ArmUp();
        take.SlideUp();
        sleep(1500);
        take.ClawOpen();
        sleep(500);
        take.ClawClose();
        sleep(500);
        take.SlideDown();
        take.ArmDown();
        sleep(1000);
        drive.followTrajectory(traj3);



    }

}

