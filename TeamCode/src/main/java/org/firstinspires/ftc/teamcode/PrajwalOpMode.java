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

        motors[0].setPower(diag1+gamepad1.right_stick_x);
        motors[1].setPower(diag2+gamepad1.right_stick_x);
        motors[2].setPower(diag2-gamepad1.right_stick_x);
        motors[3].setPower(diag1-gamepad1.right_stick_x);

        telemetry.addData("left", diag1);
        telemetry.addData("right", diag2);
    }
}
