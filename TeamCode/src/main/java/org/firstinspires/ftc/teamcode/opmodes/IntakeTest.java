package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="IntakeTest")
class IntakeTest extends OpMode {
    DcMotor intake;
    boolean state = false;
    boolean prevState = false;

    @Override
    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    @Override
    public void loop(){
        if(gamepad1.y && !prevState){
            state = !state;
            prevState = true;
            if(state){
                intake.setPower(1);
            }else{
                intake.setPower(0);
            }
        }else if(!gamepad1.y){
            prevState = false;
        }
    }
}
