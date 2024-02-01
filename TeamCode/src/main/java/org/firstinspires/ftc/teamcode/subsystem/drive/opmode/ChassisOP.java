package org.firstinspires.ftc.teamcode.subsystem.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

@Config
public class ChassisOP {
    DcMotorEx LF, RF, LB, RB;
    private IMU imu;
    double x, y, r;
    Orientation orientation;

    List<DcMotorEx> motors;
    public ChassisOP(HardwareMap hardwareMap) {
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        motors = Arrays.asList(LF, RF, LB, RB);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        imu.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        imu.resetYaw();

    }

    public void Chassis(Gamepad gamepad) {
        x = gamepad.left_stick_x;
        y = -gamepad.left_stick_y;
        r = gamepad.right_stick_x;

        orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        double theata = 0;
        theata = -orientation.thirdAngle;

        double adjustedX = x * Math.cos(theata) - y * Math.sin(theata);
        double adjustedY = x * Math.sin(theata) + y * Math.cos(theata);

        LF.setPower(adjustedY + adjustedX + r);
        RF.setPower(adjustedY - adjustedX - r);
        LB.setPower(adjustedY - adjustedX + r);
        RB.setPower(adjustedY + adjustedX - r);


    }
}