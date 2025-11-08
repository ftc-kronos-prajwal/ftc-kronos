package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        if(gamepad1.right_trigger>0.1) {
            shooterMotor.setPower(1.0);
        } else {
            shooterMotor.setPower(0.0);
        }
        telemetry.addData("Shooter Power",shooterMotor.getPower());
        telemetry.update();
    }
    public void stop(){
        shooterMotor.setPower(0.0);
    }
}