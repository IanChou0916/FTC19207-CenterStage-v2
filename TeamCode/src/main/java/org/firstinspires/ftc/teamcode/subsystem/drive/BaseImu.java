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

public class BaseImu {
    float X_axis;
    float Y_axis;
    float Z_axis;
    IMU imu;
    Orientation orientation;
    BaseChassis chassis;

    public BaseImu(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.XYZ,
                                AngleUnit.DEGREES,
                                0,
                                0,
                                0,
                                0
                        )
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
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double r = gamepad.right_stick_x;

        orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );
        
        double theata = -orientation.thirdAngle;
        
        double tmp = x;
        x = Math.cos(theata) * x - Math.sin(theata) * y;
        y = Math.sin(theata) * tmp + Math.cos(theata) * y;

        LF.setPower(y+x+r);
        LB.setPower(y-x+r);
        RF.setPower(y-x-r);
        RB.setPower(y+x-r);

        telemetry.addData("theata", Math.toDegrees(theata));
        telemetry.addData("cos", "%.2f", Math.cos(theata));
        telemetry.addData("sin", "%.2f", Math.sin(theata));
    }
}
