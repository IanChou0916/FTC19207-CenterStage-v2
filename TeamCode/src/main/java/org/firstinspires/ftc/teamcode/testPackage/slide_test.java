package org.firstinspires.ftc.teamcode.testPackage;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class slide_test {
    DcMotorEx slider;
    BNO055IMU imu;
    private boolean released = true;
    public static int limit,target;
    FtcDashboard dashboard;

    public static double
            P = 12,
            I = 0,
            D = 0,
            F = 0;


    public slide_test(HardwareMap hardwareMap) {
        limit = 1800;
        target = 0;
        dashboard = FtcDashboard.getInstance();


        slider = hardwareMap.get(DcMotorEx.class, "slide");


        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Slide(Gamepad gamepad){
        if (target >= limit){
            target = limit;
        }
        if (target < 0){
            target = 0;
        }
        target += -(int)gamepad.left_stick_y*50;
        slider.setTargetPosition(target);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPIDFCoefficients(slider.getMode(),new PIDFCoefficients(P,I,D,F));
    }


    public void dashboard(){
        slider.setTargetPosition(target);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPIDFCoefficients(slider.getMode(),new PIDFCoefficients(P,I,D,F));

        if (target >= limit){
            target = limit;
        }
        if (target < 0){
            target = 0;
        }
    }
}



