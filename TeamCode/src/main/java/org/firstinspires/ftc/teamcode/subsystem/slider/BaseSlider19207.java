package org.firstinspires.ftc.teamcode.subsystem.slider;


import static org.firstinspires.ftc.teamcode.Constants.MOTOR_LEFT_SLIDER;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_RIGHT_SLIDER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class BaseSlider19207 {
    DcMotorEx slideL, slideR;
    //R is dead wheel encoder
    FtcDashboard dashboard;
    public static double targetL = 0;
    int targetR = 0;
    public static int limit = 1200;

    private List<DcMotorEx> sliders;
    int gain = 0;
    public static int distance = 0;

    public BaseSlider19207(HardwareMap hardwareMap, int gain){

        this.gain = gain;

        slideL = hardwareMap.get(DcMotorEx.class, MOTOR_LEFT_SLIDER);
        slideR = hardwareMap.get(DcMotorEx.class, MOTOR_RIGHT_SLIDER);

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setTargetPosition(0);
        slideR.setPower(1);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Slider(Gamepad gamepad) {
        slideL.setPower(-targetL);

        slideR.setTargetPosition(targetR * gain);

        if (targetR < 0) {
            targetR = 0;
        } else if (slideR.getCurrentPosition() > limit) {
            targetR = limit/gain;//1250
        }

        targetR += gamepad.left_stick_y;
        targetL = gamepad.right_stick_y;
    }

    public void SliderCount(Gamepad gamepad) {
        double slideTheata = 180 - Math.toDegrees(slideR.getCurrentPosition() / 8192);
        double slidePhi = 90 - slideTheata;

        Math.cos(Math.toRadians(slideTheata));

        if (gamepad.a) {

        }
    }
    public void testdashboard(){
        slideR.setTargetPosition(targetR);
        slideL.setTargetPosition((int) targetL);

    }
}
