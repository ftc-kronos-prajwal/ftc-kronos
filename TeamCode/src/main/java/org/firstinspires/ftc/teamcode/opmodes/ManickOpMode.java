package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hooded_Turret_Control", group = "Turret")
public class ManickOpMode extends LinearOpMode {


    private Servo hoodServo;


    private static final double MIN_POS = 0.1;  
    private static final double MAX_POS = 0.9; 

    private double position = 0.5; 

    @Override
    public void runOpMode() {


        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

        
            if (gamepad1.right_bumper) {
                position += 0.01; 
            } else if (gamepad1.left_bumper) {
                position -= 0.01;
            }

 
            position = Math.max(MIN_POS, Math.min(MAX_POS, position));

          
            hoodServo.setPosition(position);

 
            telemetry.addData("Servo Pos", "%.2f", position);
            telemetry.update();
        }
    }
} 


