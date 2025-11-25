package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoZeroer")
public class ServoZeroer extends OpMode {
    Servo servo = null;

    @Override
    public void init(){
        servo = hardwareMap.get(Servo.class, "stopperServo");
        servo.setPosition(1.0);
    }

    @Override
    public void loop(){
    }
}
