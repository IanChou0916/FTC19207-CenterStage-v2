package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public static final DcMotor.ZeroPowerBehavior MOTOR_BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final String SERVO_INTAKE = "intake";

    public static final boolean MOTOR_CHASSIS_BRAKE = true;
    public static final boolean MOTOR_SLIDERS_BRAKE = true;
    public static final boolean MOTOR_CHASSIS_FL_REVERSED = true;
    public static final boolean MOTOR_CHASSIS_FR_REVERSED = false;
    public static final boolean MOTOR_CHASSIS_RL_REVERSED = true;
    public static final boolean MOTOR_CHASSIS_RR_REVERSED = false;

    // PID constants for motors
    public static final double[] PID_CHASSIS = {0, 0, 0};
    public static final double SLIDER_UP_KP = 0.0;
    public static final double SLIDER_DOWN_KP = 0.0;

    public static final double[] PID_INTAKE = {0, 0, 0};

    // chassis constants
    public static final boolean ENABLE_FOD = true; // field coordinate system
    public static final double CHASSIS_DEADBAND = 0.06; // for controller


}
