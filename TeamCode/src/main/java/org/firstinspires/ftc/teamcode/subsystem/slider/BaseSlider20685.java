package org.firstinspires.ftc.teamcode.subsystem.slider;

import static org.firstinspires.ftc.teamcode.Constants.MOTOR_INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_LEFT_SLIDER;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_RIGHT_SLIDER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class BaseSlider20685 {
    FtcDashboard dashboard;
    DcMotorEx slideL, slideR, slideI;
    public static int target, limit;
    public static double power = 0;
    List<DcMotorEx> motors;

    public BaseSlider20685(HardwareMap hardwareMap) {
        limit = 3000;

        dashboard = FtcDashboard.getInstance();

        slideL = hardwareMap.get(DcMotorEx.class, MOTOR_LEFT_SLIDER);
        slideR = hardwareMap.get(DcMotorEx.class, MOTOR_RIGHT_SLIDER);
        slideI = hardwareMap.get(DcMotorEx.class, MOTOR_INTAKE);

        motors = Arrays.asList(slideR, slideL);

        for (DcMotorEx motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    public void Slide(Gamepad gamepad) {
        if (target > limit) {
            target = limit;
        } else if (target < 0) {
            target = 0;
        }

        target += -1 * gamepad.right_stick_y;
        power = gamepad.left_stick_y;

        slideI.setPower(power);
        slideL.setTargetPosition(-target);
        slideR.setTargetPosition(target);
    }
}
