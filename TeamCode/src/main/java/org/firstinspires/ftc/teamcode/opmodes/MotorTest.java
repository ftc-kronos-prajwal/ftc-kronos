package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {
    DcMotor fl, fr, bl, br;
    @Override
    public void init(){
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        if(gamepad1.y){
            fr.setPower(1.0);
            //back right
        }else{
            fr.setPower(0.0);
        }
        if(gamepad1.b){
            br.setPower(1.0);
            //front right
        }else{
            br.setPower(0.0);
        }
        if(gamepad1.a){
            bl.setPower(1.0);
            //front left
        }else{
            bl.setPower(0.0);
        }
        if(gamepad1.x){
            fl.setPower(1.0);
            //back left
        }else {
            fl.setPower(0.0);
        }
    }
}
