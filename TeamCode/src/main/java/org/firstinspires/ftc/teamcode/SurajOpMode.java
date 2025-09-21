package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SurajOpMode")
public class SurajOpMode extends OpMode {

    private DcMotor[] motors = new DcMotor[4];

    private static final int front_left_motor = 0;
    private static final int back_left_motor = 1;
    private static final int back_right_motor = 2;
    private static final int front_right_motor = 3;

    @Override
    public void init() {
        motors[front_left_motor] = hardwareMap.get(DcMotor.class, "frontLeft");
        motors[back_left_motor] = hardwareMap.get(DcMotor.class, "backLeft");
        motors[back_right_motor] = hardwareMap.get(DcMotor.class, "backRight");
        motors[front_right_motor] = hardwareMap.get(DcMotor.class, "frontRight");


        motors[front_left_motor].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[back_left_motor].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[back_right_motor].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[front_right_motor].setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;


        double turn  =  gamepad1.right_stick_x;


        double leftPower  = drive + turn;
        double rightPower = drive - turn;


        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }



        motors[front_left_motor].setPower(leftPower);
        motors[back_left_motor].setPower(leftPower);
        motors[back_right_motor].setPower(rightPower);
        motors[front_right_motor].setPower(rightPower);


        telemetry.addData("Drive (Left Stick Y)", "%.2f", drive);
        telemetry.addData("Turn (Right Stick X)", "%.2f", turn);
        telemetry.addData("Left Motor Power", "%.2f", leftPower);
        telemetry.addData("Right Motor Power", "%.2f", rightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
