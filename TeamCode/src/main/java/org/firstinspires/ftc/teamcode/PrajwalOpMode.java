package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private double diag1, diag2, fl, bl, fr, br, max, leftX, rightX, leftY/*, rightY*/;

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
        leftX = gamepad1.left_stick_x*gamepad1.left_stick_x*gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x;
        leftY = gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y;
        //rightY = gamepad1.right_stick_y*gamepad1.right_stick_y*gamepad1.right_stick_y;

        diag1 = leftY + leftX;
        diag2 = leftY - leftX;

        fl = diag1-rightX;
        bl = diag2-rightX;
        fr = diag2+rightX;
        br = diag1+rightX;

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
