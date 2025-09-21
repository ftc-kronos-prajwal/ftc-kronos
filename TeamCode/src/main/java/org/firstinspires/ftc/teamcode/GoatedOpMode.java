package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// It seems Range is not used in this OpMode, you might be able to remove this import.
// import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GoatedOpMode")
public class GoatedOpMode extends OpMode {

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


        // Ensure motor directions are set correctly for Mecanum drive.
        // This configuration assumes a standard Mecanum setup where the rollers
        // on the front-left and back-right wheels form a "\" shape,
        // and the rollers on the front-right and back-left wheels form a "/" shape
        // when viewed from above.
        // If your robot moves unexpectedly, you may need to reverse some of these.
        motors[front_left_motor].setDirection(DcMotorSimple.Direction.REVERSE); // Or FORWARD
        motors[back_left_motor].setDirection(DcMotorSimple.Direction.REVERSE);  // Or FORWARD
        motors[front_right_motor].setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        motors[back_right_motor].setDirection(DcMotorSimple.Direction.FORWARD);  // Or REVERSE


        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        // Mecanum drive calculations
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Strafe Left/Right
        double rx = gamepad1.right_stick_x; // Rotation/Turning

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the [-1, 1] range.
        // Reduce the denominator slightly to ensure the wheels turn if the joystick is slightly off-center
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motors[front_left_motor].setPower(frontLeftPower);
        motors[back_left_motor].setPower(backLeftPower);
        motors[front_right_motor].setPower(frontRightPower);
        motors[back_right_motor].setPower(backRightPower);

        telemetry.addData("Left Stick Y (Forward/Backward)", "%.2f", y);
        telemetry.addData("Left Stick X (Strafe)", "%.2f", x);
        telemetry.addData("Right Stick X (Turn)", "%.2f", rx);
        telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
        telemetry.addData("Back Left Power", "%.2f", backLeftPower);
        telemetry.addData("Front Right Power", "%.2f", frontRightPower);
        telemetry.addData("Back Right Power", "%.2f", backRightPower);
        telemetry.addData("Denominator", "%.2f", denominator);
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
