package org.firstinspires.ftc.teamcode.testPackage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@Autonomous(name = "redBack",group = "Auto")
public class roadrunnertest_red1 extends LinearOpMode {
    Trajectory traj1, traj2, traj3, traj4;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake take = new Intake(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -58, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36, -18.5), Math.toRadians(90))
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end())
                .back(69.35)
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(32)
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(31)
                .build();


        waitForStart();

        if (isStopRequested()) return ;


        drive.followTrajectory(traj1);
        take.ClawtrunDown();
        sleep(500);
        drive.turn(Math.toRadians(94));
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

    }
}
