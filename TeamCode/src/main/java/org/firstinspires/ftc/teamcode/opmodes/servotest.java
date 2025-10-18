package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Test")
public class servotest extends LinearOpMode {

    // Declare the servo and a timer
    private CRServo continuousServo;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        // IMPORTANT: "servo1" must match the name in your robot's configuration file
        continuousServo = hardwareMap.get(CRServo.class, "servo1");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Reset the timer once the OpMode starts
        runtime.reset();

        // Spin the servo to the right for 10 seconds
        while (opModeIsActive() && runtime.seconds() < 10.0) {
            // Setting power to 1.0 makes a CR servo spin forward (right)
            continuousServo.setPower(1.0);

            // Display the elapsed time on the Driver Station
            telemetry.addData("Action", "Spinning Right");
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.update();
        }

        // Stop the servo
        continuousServo.setPower(0.0);

        // Print a success message to the telemetry
        telemetry.addData("Status", "Successful");
        telemetry.addData("Action", "Stopped");
        telemetry.update();

        // Keep the OpMode running until the user presses STOP
        // This allows the "Successful" message to stay on the screen
        while (opModeIsActive()) {
            // The op mode is waiting, do nothing.
            // Using idle() helps manage system resources. [15]
            idle();
        }
    }
}
