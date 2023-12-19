package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class op19207 extends robotBase {
    @Override
    protected void robotInit() {
        waitForStart();
    }
    @Override
    protected void robotStart() throws InterruptedException {
        while (opModeIsActive()) {
            drive.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if (gamepad1.a) {
                sliders.setSliderPower(1);
            } else if (gamepad1.b) {
                sliders.setSliderPower(-1);
            } else {
                sliders.setSliderPower(0);
            }


        }
    }
}
