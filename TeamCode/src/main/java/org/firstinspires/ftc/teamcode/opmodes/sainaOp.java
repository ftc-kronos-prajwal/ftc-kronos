package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "sainaOp")
public class sainaOp extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float leftY, rightY;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("left_Motor");
        rightMotor = hardwareMap.dcMotor.get("right_Motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;
            leftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "left="+leftY+"right="+rightY);
            telemetry.update();

            idle();
        }
    }





}
