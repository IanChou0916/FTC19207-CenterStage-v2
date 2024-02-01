package org.firstinspires.ftc.teamcode.subsystem.slider;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class drone_test{
    FtcDashboard dashboard;
    double emission;
    public static double Drone;
    Servo drone;


    public drone_test(HardwareMap hardwareMap){
        dashboard = FtcDashboard.getInstance();
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);
        drone.setPosition(0);

    }


    public void drone(Gamepad gamepad){
        emission = 0.9;
        if (gamepad.b) {
            drone.setPosition(emission);

        }
    }
    public void dashboard(){
        drone.setPosition(Drone);

    }

}
