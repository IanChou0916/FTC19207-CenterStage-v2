package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BaseImu19207 {
    float X_axis;
    float Y_axis;
    float Z_axis;
    public IMU imu;
    Orientation orientation;

    public BaseImu19207(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        imu.resetYaw();
    }

    public void ReadImu(Telemetry telemetryout) {
        Telemetry telemetry = new MultipleTelemetry(telemetryout, FtcDashboard.getInstance().getTelemetry());

        orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        X_axis = orientation.firstAngle;
        Y_axis = orientation.secondAngle;
        Z_axis = orientation.thirdAngle;

        telemetry.addData("X_axis(radian)", "%.2f", X_axis);
        telemetry.addData("Y_axis(radian)", "%.2f", Y_axis);
        telemetry.addData("Z_axis(radian)", "%.2f", Z_axis);
    }

    public void AbsMovingIMU(DcMotorEx LF, DcMotorEx RF, DcMotorEx LB, DcMotorEx RB, Gamepad gamepad, Telemetry telemetry) {
        ReadImu(telemetry);

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double r = gamepad.right_stick_x;

        orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        double theata = 0;
        theata = -orientation.thirdAngle;


        double tmp = x;
        x = Math.cos(theata) * x - Math.sin(theata) * y;
        y = Math.sin(theata) * tmp + Math.cos(theata) * y;

        LF.setPower(y+x+r);
        LB.setPower(y-x+r);
        RF.setPower(y-x-r);
        RB.setPower(y+x-r);

        telemetry.addData("theata", Math.toDegrees(theata));
        telemetry.addData("cos", Math.cos(theata));
        telemetry.addData("sin", Math.sin(theata));

    }
    public void ResetIMU(Gamepad gamepad) {
        if (gamepad.a) {
            imu.resetYaw();
        }
    }
}

