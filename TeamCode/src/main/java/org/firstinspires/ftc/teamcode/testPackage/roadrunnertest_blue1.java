package org.firstinspires.ftc.teamcode.testPackage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Autonomous(name = "blueBack",group = "Auto")
@Config
public class roadrunnertest_blue1 extends LinearOpMode {
    Trajectory traj1,traj2,traj3,traj4;

    @Override
    public void runOpMode() throws InterruptedException{
        Intake take = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, 56, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36,12),Math.toRadians(270))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .back(70  )
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(26)
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(24)
                .build();
        waitForStart();

        if(isStopRequested());


        drive.followTrajectory(traj1);
        take.ClawtrunDown();
        sleep(500);
        drive.turn(Math.toRadians(-96));
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
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
        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(96*2));


    }

}

