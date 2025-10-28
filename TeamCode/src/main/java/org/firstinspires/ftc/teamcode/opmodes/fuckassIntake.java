package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="fuckassIntake")
public class fuckassIntake extends OpMode {
    private DcMotor intakeMotor;
    Servo servo = null;

    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.x) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.y) {
            servo.setPosition(1.0);
        }
        else if (gamepad1.a) {
            servo.setPosition(0);
        }

        telemetry.addData("Servo position", servo.getPosition());
        telemetry.update();

        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();
    }
}