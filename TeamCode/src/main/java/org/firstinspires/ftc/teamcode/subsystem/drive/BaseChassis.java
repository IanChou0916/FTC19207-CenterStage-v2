package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Arrays;
import java.util.List;

public class BaseChassis {
    public DcMotorEx LF = null;
    public DcMotorEx RF = null;
    public DcMotorEx RB = null;
    public DcMotorEx LB = null;
    List<DcMotorEx> motors;
    BaseImu imu;

    public BaseChassis (HardwareMap hardwareMap) {
        imu = new BaseImu(hardwareMap);
        
        Constants constants = new Constants();
        LF = hardwareMap.get(DcMotorEx.class, constants.MOTOR_CHASSIS_FL);
        RF = hardwareMap.get(DcMotorEx.class, constants.MOTOR_CHASSIS_FR);
        LB = hardwareMap.get(DcMotorEx.class, constants.MOTOR_CHASSIS_RL);
        RB = hardwareMap.get(DcMotorEx.class, constants.MOTOR_CHASSIS_RR);
        motors = Arrays.asList(LF, RF, LB, RB);

        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        imu.imu.resetYaw();
    }

    public void Drive(Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double r = gamepad.right_trigger - gamepad.left_trigger;

        LF.setPower(y+x+r);
        LB.setPower(y-x+r);
        RF.setPower(y-x-r);
        RB.setPower(y+x-r);
    }

    public void AbsMoving(Gamepad gamepad, Telemetry telemetry) {
        imu.AbsMovingIMU(LF, RF, LB, RB, gamepad, telemetry);
    }
}
