package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Turret Rotation", group = "TeleOp")
public class TurretRotation extends OpMode {
    private CRServo leftServo;
    private CRServo rightServo;

    public void init(){
        leftServo = hardwareMap.get(CRServo.class, "leftservo");
        rightServo = hardwareMap.get(CRServo.class, "rightservo");
        telemetry.addLine("Rotation ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = 0.0;
        if (gamepad1.right_trigger > 0) {
            power = -gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0){
            power = gamepad1.left_trigger;
        }
        leftServo.setPower(power);
        rightServo.setPower(power);

        telemetry.addData("Turret Power", power);
        telemetry.update();
    }
    public void stop(){
        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}