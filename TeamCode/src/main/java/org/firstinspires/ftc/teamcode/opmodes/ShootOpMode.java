package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ShootOpMode", group = "Examples")
public class ShootOpMode extends OpMode {
    private DcMotor shooterMotor;

    public void init() {
        shooterMotor = hardwareMap.get(DcMotor.class,"shootermotor");
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Intialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        shooterMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
    }
    public void stop(){
        shooterMotor.setPower(0.0);
    }
}