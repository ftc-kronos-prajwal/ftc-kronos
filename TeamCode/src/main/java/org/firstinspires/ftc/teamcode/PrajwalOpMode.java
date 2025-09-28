package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private float diag1, diag2, fl, bl, fr, br, max;

    @Override
    public void init(){
        motors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
        motors[1] = hardwareMap.get(DcMotor.class,"backLeft");
        motors[2] = hardwareMap.get(DcMotor.class,"frontRight");
        motors[3] = hardwareMap.get(DcMotor.class,"backRight");

        for(int i = 0; i < motors.length; i+=1) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection((i < 2) ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            //motors[i].setDirection(DcMotor.Direction.FORWARD);
        }
    }

    @Override
    public void loop(){
        diag1 = gamepad1.left_stick_y + gamepad1.left_stick_x;
        diag2 = gamepad1.left_stick_y - gamepad1.left_stick_x;

        fl = diag1-gamepad1.right_stick_x;
        bl = diag2-gamepad1.right_stick_x;
        fr = diag2+gamepad1.right_stick_x;
        br = diag1+gamepad1.right_stick_x;

        max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
        if(max > 1){
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        motors[0].setPower(fl);
        motors[1].setPower(bl);
        motors[2].setPower(fr);
        motors[3].setPower(br);

        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl);
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);
        telemetry.update();
    }
}
