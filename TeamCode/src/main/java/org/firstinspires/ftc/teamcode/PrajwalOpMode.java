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
        float rightAmount = gamepad1.left_stick_y, leftAmount = gamepad1.left_stick_y;
        float xFactor = gamepad1.left_stick_x*2-1;
        rightAmount *= xFactor;
        leftAmount *= xFactor;
        if(Math.abs(gamepad1.left_stick_y) <= 0.1 && Math.abs(gamepad1.left_stick_x) >= 0.1){
            rightAmount = gamepad1.left_stick_x;
            leftAmount = -gamepad1.left_stick_x;
        }

        motors[0].setPower(leftAmount);
        motors[1].setPower(leftAmount);
        motors[2].setPower(rightAmount);
        motors[3].setPower(rightAmount);

        telemetry.addData("left", leftAmount);
        telemetry.addData("right", rightAmount);
    }
}
