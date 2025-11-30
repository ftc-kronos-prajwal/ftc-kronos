package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor test_color;

    private float gain = 0.0f;
    private boolean pressed = false;
    @Override
    public void runOpMode() {
        test_color = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !pressed){
                gain += 0.5;
                test_color.setGain(gain);
                pressed = true;
            }else if(!gamepad1.a){
                pressed = false;
            }

            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            NormalizedRGBA colors = test_color.getNormalizedColors();

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            telemetry.addData("Gain", gain);
            telemetry.update();
        }
    }
}
