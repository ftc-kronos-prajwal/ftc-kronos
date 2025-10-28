package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;
import java.util.List;

@TeleOp(name = "Hooded_Turret_Control", group = "Turret")
public class Hooded_Turret_Control extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo hoodServo;
    private static final double MIN_POS = 0.1;
    private static final double MAX_POS = 0.9;
    private double hoodPos = 0.5;

    @Override
    public void runOpMode() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        initAprilTag();

        telemetry.addLine("Ready — press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection detection = detections.get(0);

                if (detection.ftcPose != null) {
                    double distance = detection.ftcPose.range;
                    hoodPos = mapRange(distance, 10, 60, MAX_POS, MIN_POS);
                    hoodPos = Math.max(MIN_POS, Math.min(MAX_POS, hoodPos));
                    hoodServo.setPosition(hoodPos);

                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Distance (in)", "%.1f", distance);
                    telemetry.addData("Hood Pos", "%.2f", hoodPos);
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1612.409975, 1594.66826, 967.3439456, 495.7939473)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        }
        builder.setCameraResolution(new Size(1280, 720));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double mapRange(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}

