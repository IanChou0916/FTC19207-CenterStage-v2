package org.firstinspires.ftc.teamcode.testPackage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.opmode.ChassisOP;
import org.firstinspires.ftc.teamcode.subsystem.slider.Claw;
import org.firstinspires.ftc.teamcode.subsystem.slider.drone_test;

@TeleOp
public class OP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisOP drive = new ChassisOP(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        slide_test slider = new slide_test(hardwareMap);
        arm_test arm = new arm_test(hardwareMap);
        drone_test Drone = new drone_test(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.Chassis(gamepad1);
            claw.ClawControl(gamepad2);
            slider.Slide(gamepad2);
            arm.Arm(gamepad1);
            Drone.drone(gamepad1);
        }


    }
}

