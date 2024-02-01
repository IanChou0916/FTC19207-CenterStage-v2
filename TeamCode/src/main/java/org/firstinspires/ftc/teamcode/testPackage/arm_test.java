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
public class arm_test {
    DcMotorEx arm;
    BNO055IMU imu;
    private boolean released = true;
    public static int target;
    FtcDashboard dashboard;

    public static double
            P = 1.65,
            I = 0,
            D = 0,
            F = 0;





    public arm_test(HardwareMap hardwareMap) {
        target = 0;
        dashboard = FtcDashboard.getInstance();


        arm = hardwareMap.get(DcMotorEx.class, "arm");


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Arm(Gamepad gamepad){
        if(gamepad.left_bumper){
            target = 770;
            arm.setTargetPosition(target);
        }else if(gamepad.right_bumper){
            target = 0;
            arm.setTargetPosition(target);
        }
        arm.setPower(0.4);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPIDFCoefficients(arm.getMode(),new PIDFCoefficients(P,I,D,F));

    }


    public void dashboard(){
        arm.setTargetPosition(target);
        arm.setPower(0.3);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPIDFCoefficients(arm .getMode(),new PIDFCoefficients(P,I,D,F));

    }
}



