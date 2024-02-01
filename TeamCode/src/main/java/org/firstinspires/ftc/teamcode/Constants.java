package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.signedness.qual.Constant;

@Constant
@Config
public class Constants {
    private static final boolean DEBUG = false;
    // motor names in Hardware map
    public static final String MOTOR_CHASSIS_FL = "LF";
    public static final String MOTOR_CHASSIS_FR = "RF";
    public static final String MOTOR_CHASSIS_RL = "LB";
    public static final String MOTOR_CHASSIS_RR = "RB";

    public static final String MOTOR_LEFT_SLIDER = "SL";
    public static final String MOTOR_RIGHT_SLIDER = "SR";
    public static final String MOTOR_INTAKE = "SI";
    public static final String SERVO_INTAKE = "intake";
    public static final String RGB_SERVO = "RGB";
    public static final DcMotor.ZeroPowerBehavior MOTOR_CHASSIS_BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotor.ZeroPowerBehavior MOTOR_SLIDERS_BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final double ClawLClose = 0.7;
    public static final double ClawRClose = 0.2;
    public static final double ClawTrunDown = 1;
    public static final double ClawLOpen = 0.5;
    public static final double ClawROpen = 0.0;
    public static final double ClawTrunUp = 0.0;


    // PID constants for motors
    public static final double[] PID_CHASSIS = {0, 0, 0};
    public static final double SLIDER_UP_KP = 0.0;
    public static final double SLIDER_DOWN_KP = 0.0;


    public static final double[] PID_INTAKE = {0, 0, 0};


    // chassis constants
    public static final boolean ENABLE_FOD = true; // field coordinate system
    public static final double CHASSIS_DEADBAND = 0.06; // for controller
    
    public static int desiredTagId = 0;

}
