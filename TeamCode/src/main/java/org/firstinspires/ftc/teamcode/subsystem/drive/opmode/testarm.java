package org.firstinspires.ftc.teamcode.subsystem.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public abstract class testarm  {
    FtcDashboard dashboard;
    Servo servoL,servoR,servoMid;
    public static double targetCatchL = 0, targetCatchR = 0, targetMid = 0;

    public testarm(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();

        servoL = hardwareMap.get(Servo.class,"servoL");
        servoR = hardwareMap.get(Servo.class,"servoR");
        servoMid = hardwareMap.get(Servo.class,"servoMid");

        servoL.setDirection(Servo.Direction.FORWARD);
        servoR.setDirection(Servo.Direction.REVERSE) ;
        servoMid.setDirection(Servo.Direction.REVERSE);




        servoMid.setPosition(0);

    }
    public void arm(Gamepad gamepad,double turnDegree_close  ,double ArmTurnDegree_down ,double ArmTurnDegree_up ) {
        boolean true_false=false;
        boolean true_false1=false;


        if (!true_false) {
            servoL.setPosition(0);
            servoR.setPosition(0);
        }
        if ((gamepad.a) && (true_false== false)) {

            servoL.setDirection(Servo.Direction.REVERSE);
            servoR.setDirection(Servo.Direction.FORWARD);

            servoL.setPosition(0);
            servoR.setPosition(0);
            true_false = true;
        }
        if (gamepad.b) {

            servoL.setDirection(Servo.Direction.FORWARD);
            servoR.setDirection(Servo.Direction.REVERSE);

            servoL.setPosition(turnDegree_close);
            servoR.setPosition(turnDegree_close);
            true_false = false;
        }
        if(((gamepad.x) && true_false1) == false){
            servoMid.setPosition(ArmTurnDegree_up/180);
            true_false1 = true;
        }
        if(gamepad.y){
            servoMid.setPosition(ArmTurnDegree_down/180);
        }


    }

    public void servoCach() {
        servoR.setPosition(targetCatchL);
        servoL.setPosition(targetCatchR);
    }

    public void servoSpin() {
        servoMid.setPosition(targetMid);
    }
}
