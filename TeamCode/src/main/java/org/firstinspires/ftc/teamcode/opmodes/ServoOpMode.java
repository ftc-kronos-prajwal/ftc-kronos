package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoOpMode")
public class ServoOpMode extends OpMode {
    Servo servo = null;

    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0);
    }

    @Override
    public void loop(){
        if (gamepad1.y) {
            servo.setPosition(1.0);
        }
        else if (gamepad1.a) {
            servo.setPosition(0);
        }
        telemetry.addData("Servo position", servo.getPosition());
        telemetry.update();
        }
}
