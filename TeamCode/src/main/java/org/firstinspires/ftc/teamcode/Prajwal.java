package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SimpleRegularOpMode")
public class Prajwal extends OpMode {
    private DcMotor[4] motors;


    @Override
    public void init(){
        motors[0] = hardwareMap.get("frontLeft");
        motors[1] = hardwareMap.get("backLeft");
        motors[2] = hardwareMap.get("frontRight");
        motors[3] = hardwareMap.get("backRight");

        for(int i = 0; i < motors.length; i += 1){
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        System.out.println("init done");
    }
    @Override
    public void init_loop(){
        g
        System.out.println("init_loop");
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
        System.out.println("loop");
    }
}
