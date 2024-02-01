package org.firstinspires.ftc.teamcode.subsystem.intake;

import static org.firstinspires.ftc.teamcode.Constants.ClawLClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawLOpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawRClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawROpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunDown;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunUp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Intake{
    static boolean clickRightBumper = false;
    static boolean clickLeftBumper = false;
    static boolean clawRightState = false;
    static boolean clawLeftState = false;

    static Servo servoL;
    static Servo servoR;
    static Servo servoMid;

    static DcMotorEx slider;
    static DcMotorEx arm;
    public Intake(HardwareMap hardwareMap){
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoMid = hardwareMap.get(Servo.class, "servoMid");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slider = hardwareMap.get(DcMotorEx.class, "slide");

        servoL.setDirection(Servo.Direction.FORWARD);
        servoR.setDirection(Servo.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setPower(1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoR.setPosition(0.2);
        servoL.setPosition(0.7);
    }
    public static void ClawOpen() {

            servoL.setPosition(ClawLOpen);
            servoR.setPosition(ClawROpen);

    }
    public static void ClawClose(){

        servoL.setPosition(ClawLClose);
        servoR.setPosition(ClawRClose);
    }
    public static void ClawtrunDown(){

        servoMid.setPosition(ClawTrunDown);
    }
    public static void ClawtrunUp(){

        servoMid.setPosition(ClawTrunUp);
    }
    public static void ClawRClose(){

        servoR.setPosition(ClawRClose);
    }
    public static void ClawROpen() {

        servoR.setPosition(ClawROpen);
    }
    public static void ClawLClose(){
        servoL.setPosition(ClawLClose);
    }
    public static void ClawLOpen(){

        servoL.setPosition(ClawLOpen);
    }
    public static void ArmUp(){
        arm.setTargetPosition(770);
        arm.setPower(0.5);
    }
    public static void ArmDown(){
        arm.setTargetPosition(0);
        arm.setPower(0.5);
    }
    public static void SlideUp(){
        slider.setTargetPosition(850);
        slider.setPower(1);
    }
    public static void SlideDown(){
        slider.setTargetPosition(0);
        slider.setPower(1);
    }
    public static void SlideUp2(){
        slider.setTargetPosition(1500);
        slider.setPower(1);
    }

}
