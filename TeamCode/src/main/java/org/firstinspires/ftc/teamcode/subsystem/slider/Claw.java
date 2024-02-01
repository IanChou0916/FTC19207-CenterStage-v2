package org.firstinspires.ftc.teamcode.subsystem.slider;

import static org.firstinspires.ftc.teamcode.Constants.ClawLClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawLOpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawRClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawROpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunDown;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunUp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Claw {
    static boolean clickRightBumper = false;
    static boolean clickLeftBumper = false;
    static boolean clawRightState = false;
    static boolean clawLeftState = false;

    Servo servoL, servoR, servoMid;

    public Claw(HardwareMap hardwareMap){
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoMid = hardwareMap.get(Servo.class, "servoMid");
        servoL.setDirection(Servo.Direction.FORWARD);
        servoR.setDirection(Servo.Direction.REVERSE);
        servoMid.setPosition(0);
        servoL.setPosition(0.5);
        servoR.setPosition(0);
    }
    public void ClawControl(Gamepad gamepad){
        if(gamepad.a) {
            servoL.setPosition(ClawLClose);
            servoR.setPosition(ClawRClose);
        }

        if(gamepad.b) {
            servoL.setPosition(ClawLOpen);
            servoR.setPosition(ClawROpen);
        }

        if(gamepad.x){
            servoMid.setPosition(ClawTrunDown);
        }
        if(gamepad.y){
            servoMid.setPosition(ClawTrunUp);
        }
        if (gamepad.right_bumper && clickRightBumper == false) {
            clickRightBumper = true;

            if (clawRightState == false) {
                servoR.setPosition(ClawRClose );
                clawRightState = true;
            } else {
                servoR.setPosition(ClawROpen);
                clawRightState = false;
            }

        } else if (gamepad.right_bumper == false) {
            clickRightBumper = false;
        }

        if (gamepad.left_bumper && clickLeftBumper == false) {
            clickLeftBumper = true;

            if (clawLeftState == false) {
                servoL.setPosition(ClawLClose);
                clawLeftState = true;
            } else {
                servoL.setPosition(ClawLOpen);
                clawLeftState = false;
            }

        } else if (gamepad.left_bumper == false) {
            clickLeftBumper = false;
        }


    }
}



