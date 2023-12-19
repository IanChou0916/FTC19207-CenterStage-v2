package org.firstinspires.ftc.teamcode.subsystem.slider;


import static org.firstinspires.ftc.teamcode.Constants.MOTOR_LEFT_SLIDER;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_RIGHT_SLIDER;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_SLIDERS_BRAKE;
import static org.firstinspires.ftc.teamcode.Constants.SLIDER_UP_KP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class slider {
    DcMotorEx SliderL = null;
    DcMotorEx SliderR = null;

    private List<DcMotorEx> sliders;
    public slider(HardwareMap hardwareMap){
        SliderL = hardwareMap.get(DcMotorEx.class,MOTOR_LEFT_SLIDER);
        SliderR = hardwareMap.get(DcMotorEx.class,MOTOR_RIGHT_SLIDER);
        sliders = Arrays.asList(SliderL,SliderR);

        for (DcMotorEx motor : sliders) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
        for (DcMotorEx motor : sliders) {
            motor.setZeroPowerBehavior(MOTOR_SLIDERS_BRAKE);
        }
        for (DcMotorEx motor : sliders) {
            motor.setPositionPIDFCoefficients(SLIDER_UP_KP);
        }
    }
    public void setSliderPower(double power){
        SliderL.setPower(power);
        SliderR.setPower(power);
    }

}
