package org.firstinspires.ftc.teamcode.subsystem.intake;

import static org.firstinspires.ftc.teamcode.Constants.MOTOR_LEFT_SLIDER;
import static org.firstinspires.ftc.teamcode.Constants.MOTOR_RIGHT_SLIDER;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    DcMotorEx intake = null;
    ColorSensor RGB = null;
    public intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        RGB = hardwareMap.get(ColorSensor.class,"RGB");
    }
}
