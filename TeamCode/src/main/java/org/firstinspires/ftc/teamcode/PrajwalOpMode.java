package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];

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

        System.out.println("init done");
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        System.out.println("start");
    }
    @Override
    public void stop(){
        System.out.println("stop");
    }
    @Override
    public void loop(){
        float diag1 = gamepad1.left_stick_y + gamepad1.left_stick_x, diag2 = gamepad1.left_stick_y - gamepad1.left_stick_x;

        float fl = diag1-gamepad1.right_stick_x;
        float bl = diag2-gamepad1.right_stick_x;
        float fr = diag2+gamepad1.right_stick_x;
        float br = diag1+gamepad1.right_stick_x;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
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
