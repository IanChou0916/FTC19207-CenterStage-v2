package org.firstinspires.ftc.teamcode.testPackage;

import static org.firstinspires.ftc.teamcode.Constants.ClawLClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawLOpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawRClose;
import static org.firstinspires.ftc.teamcode.Constants.ClawROpen;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunDown;
import static org.firstinspires.ftc.teamcode.Constants.ClawTrunUp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
public class ClawTest extends LinearOpMode {
    Servo servoL, servoR, servoMid;
    public static double ClawL = 0.5;
    public static double CLawR = 0.0;
    public static double ClawTurn = 1.0;
    boolean clickRightBumper = false, clickLeftBumper = false;
    boolean clawRightState = false;
    boolean clawLeftState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoMid = hardwareMap.get(Servo.class, "servoMid");

        servoL.setDirection(Servo.Direction.FORWARD);
        servoR.setDirection(Servo.Direction.REVERSE);
        servoMid.setDirection(Servo.Direction.REVERSE);
        servoMid.setPosition(1);
        servoL.setPosition(0.5);
        servoR.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            /*
            servoMid.setPosition(ClawTurn);
            servoL.setPosition(ClawL);
            servoR.setPosition(CLawR);
             */
            telemetry.addData("servoL", ClawL);
            telemetry.addData("servoR", CLawR);
            telemetry.addData("servoTurn", ClawTurn);
            telemetry.update();
            if (gamepad2.a) {
                servoL.setPosition(ClawLOpen);
                servoR.setPosition(ClawROpen);
            }
            if (gamepad2.b) {
                servoL.setPosition(ClawLClose);
                servoR.setPosition(ClawRClose);
            }
            if (gamepad2.y) {
                servoMid.setPosition(ClawTrunUp);
            }
            if (gamepad2.x) {
                servoMid.setPosition(ClawTrunDown);
            }
            if (gamepad2.right_bumper && clickRightBumper == false) {
                clickRightBumper = true;

                if (clawRightState == false) {
                    servoR.setPosition(ClawLClose);
                    clawRightState = true;
                } else {
                    servoR.setPosition(ClawLOpen);
                    clawRightState = false;
                }

            } else if (gamepad2.right_bumper == false) {
                clickRightBumper = false;
            }

            if (gamepad2.left_bumper && clickLeftBumper == false) {
                clickLeftBumper = true;

                if (clawLeftState == false) {
                    servoL.setPosition(ClawRClose);
                    clawLeftState = true;
                } else {
                    servoL.setPosition(ClawROpen);
                    clawLeftState = false;
                }

            } else if (gamepad2.left_bumper == false) {
                clickLeftBumper = false;
            }
            //if(gamepad2.right)



        }

    }
}
