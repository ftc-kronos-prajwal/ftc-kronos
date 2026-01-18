package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@TeleOp(name="MainOpMode")
public class MainOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private DcMotor intakeMotor, turretLaunchMotor, kickerMotor;

    private Servo intakeServo, stopperServo;

    private double diag1, diag2, fl, bl, fr, br, max, leftX, rightX, leftY/*, rightY*/, intakeServoPosition = 0.6, stopperServoPosition = 0.8, turretRotPower = 0, turretLaunchPower = 0, kickerMotorPower = 0.0;
    private long lastUpdateTime, lastIntakeTime, lastTurretRotTime, currentTime;
    private boolean lastTurretRotLogged = false, pLBState = false, pRBState = false;

    private short lastTurretRotDir = 0;
    private Servo leftServo, rightServo;

    private ElapsedTime timer = new ElapsedTime(), turretRotationTimer = new ElapsedTime(), lastCall = new ElapsedTime(), lastAprilTagDetection = new ElapsedTime();

    private NormalizedColorSensor colorSensor;

    private SampleMecanumDrive drive;
    private TrajectorySequence trajectory;

    private boolean forceIntake = false, turretRotated = false;

    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal visionPortal;

    private double currentServoPosition = 0.0;

    private boolean redAliance = false;

    private final int RED_ALIANCE_GOAL_APRIL_TAG_ID = 24, BLUE_ALIANCE_GOAL_APRIL_TAG_ID = 20;
    private int targetGoalAprilTagID;

    private double[] rot(double x, double y, double theta) {
        double[] res = new double[2];
        double cos = Math.cos(theta), sin = Math.sin(theta);
        res[0] = x * cos - y * sin;
        res[1] = x * sin + y * cos;
        return res;
    }

    @Override
    public void init(){
        motors[0] = hardwareMap.get(DcMotor.class, "frontLeft");
        motors[1] = hardwareMap.get(DcMotor.class,"backLeft");
        motors[2] = hardwareMap.get(DcMotor.class,"frontRight");
        motors[3] = hardwareMap.get(DcMotor.class,"backRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hardwareMap.get(Servo.class, "servo");
        intakeServo.setPosition(intakeServoPosition);

        stopperServo = hardwareMap.get(Servo.class, "stopperServo");
        stopperServo.setPosition(stopperServoPosition);

        kickerMotor = hardwareMap.get(DcMotor.class, "kickerMotor");
        kickerMotor.setPower(kickerMotorPower);

        for(int i = 0; i < motors.length; i+=1) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(0, 0, 0);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftServo = hardwareMap.get(Servo.class, "leftservo");
        rightServo = hardwareMap.get(Servo.class, "rightservo");
        turretLaunchMotor = hardwareMap.get(DcMotor.class,"shootermotor");

        turretLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        rightServo.setPosition(currentServoPosition);
        leftServo.setPosition(currentServoPosition);

        //visionportal setup
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(2013.92508, 2009.041219, 921.8679165, 451.7447623)
                .build();

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.setCameraResolution(new Size(1920, 1080));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);
        builder.addProcessor(colorLocator);

        visionPortal = builder.build();

        lastCall.reset();
    }

    @Override
    public void init_loop(){
        if(gamepad1.x){
            redAliance = false;
        }else if(gamepad1.b){
            redAliance = true;
        }

        telemetry.addData("Aliance: " , redAliance ? "red" : "blue");
        telemetry.update();
    }

    @Override
    public void start(){
        targetGoalAprilTagID = redAliance ? RED_ALIANCE_GOAL_APRIL_TAG_ID : BLUE_ALIANCE_GOAL_APRIL_TAG_ID;
    }
    @Override
    public void loop(){
        //drivetrain

        Pose2d estimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-estimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        if(!drive.isBusy()) {
            //turret
            currentTime = System.currentTimeMillis();

            /*if (currentTime - lastUpdateTime > 50) {
                int currentDir = gamepad1.left_bumper ? 1 : (gamepad1.right_bumper ? -1 : 0);

                if (currentDir != 0) {
                    if (lastTurretRotDir != currentDir) {
                        turretRotPower = 0;
                        lastTurretRotDir = (short) currentDir;
                    }

                    double elapsed = (double) (currentTime - lastTurretRotTime);
                    double rampIncrement = (lastTurretRotLogged) ? (elapsed / 500.0) : 0.5;

                    turretRotPower += (currentDir * rampIncrement);

                    turretRotPower = Math.max(-1.0, Math.min(1.0, turretRotPower));

                    lastTurretRotTime = currentTime;
                    lastTurretRotLogged = true;
                } else {
                    turretRotPower = 0;
                    lastTurretRotDir = 0;
                    lastTurretRotLogged = false;
                }
            }*/
            if(gamepad1.left_bumper == gamepad1.right_bumper){
                if(turretRotationTimer.milliseconds() > 5000) {
                    turretRotated = false;
                }
            }else {
                currentServoPosition += gamepad1.left_bumper ? (double) lastCall.nanoseconds() /100000000 : (double) -lastCall.nanoseconds() /100000000;
                currentServoPosition = Range.clip(currentServoPosition, 0.0, 0.4);

                leftServo.setPosition(currentServoPosition);
                rightServo.setPosition(currentServoPosition);
                turretRotated = true;
                turretRotationTimer.reset();
            }

            turretLaunchMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            /*if(Math.abs(gamepad1.right_trigger-gamepad1.left_trigger) >= 0.95){
                if(timer.milliseconds() > 2000){
                    if(stopperServoPosition != 0.7){
                        stopperServo.setPosition(0.7);
                        stopperServoPosition = 0.7;
                    }
                    intakeMotor.setPower(0.0);
                }else if(timer.milliseconds() > 1000){
                    if(stopperServoPosition != 0.9){
                        stopperServo.setPosition(0.9);
                        stopperServoPosition = 0.9;
                    }
                    if(kickerMotorPower != 1.0){
                        kickerMotor.setPower(1.0);
                        kickerMotorPower = 1.0;
                    }
                    intakeMotor.setPower(-1.0);
                    forceIntake = true;
                }
            }else{
                timer.reset();
                if(stopperServoPosition != 0.9){
                    stopperServo.setPosition(0.9);
                    stopperServoPosition = 0.9;
                    intakeMotor.setPower(0.0);
                }
                if(kickerMotorPower != 0.0){
                    kickerMotor.setPower(0.0);
                    kickerMotorPower = 0.0;
                }
                forceIntake = false;
            }*/

            if(Math.abs(gamepad1.right_trigger-gamepad1.left_trigger) >= 0.95){
                if(timer.milliseconds() > 6000){
                    intakeMotor.setPower(0.0);
                }else if(timer.milliseconds() > 5000){
                    if(kickerMotorPower != 1.0){
                        kickerMotor.setPower(1.0);
                        kickerMotorPower = 1.0;
                    }
                    intakeMotor.setPower(-1.0);
                    forceIntake = true;
                }
            }else{
                timer.reset();
                if(kickerMotorPower != 0.0){
                    kickerMotor.setPower(0.0);
                    kickerMotorPower = 0.0;
                }
                forceIntake = false;
            }

            /*Pose2d estimate = drive.getPoseEstimate();

            double[] rotated = rot(gamepad1.left_stick_x, gamepad1.left_stick_y, estimate.getHeading());
            double y = -rotated[1];  // Forward/backward (inverted)
            double x = rotated[0];   // Strafe left/right
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers using mecanum drive kinematics
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            // Find the maximum power
            double maxPower = Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))
            );

            // Normalize powers if any exceed 1.0
            if (maxPower > 1.0) {
                fl /= maxPower;
                fr /= maxPower;
                bl /= maxPower;
                br /= maxPower;
            }

            motors[0].setPower(fl);
            motors[1].setPower(bl);
            motors[2].setPower(fr);
            motors[3].setPower(br);

            telemetry.addLine("-------DRIVETRAIN------");
            telemetry.addData("fl", fl);
            telemetry.addData("bl", bl);
            telemetry.addData("fr", fr);
            telemetry.addData("br", br);
            telemetry.addData("heading", estimate.getHeading());
            telemetry.update();*/

            //intake
            currentTime = System.currentTimeMillis();
            if (gamepad1.a) {
                if(stopperServoPosition != 0.9){
                    stopperServo.setPosition(0.9);
                    stopperServoPosition = 0.9;
                    try{
                        Thread.sleep(50);
                    }catch(InterruptedException e){
                        Thread.currentThread().interrupt();
                    }
                }
                intakeMotor.setPower(-1.0);
                if (intakeServoPosition != 0.3) {
                    intakeServo.setPosition(0.3);
                    intakeServoPosition = 0.3;
                }
            } else if (gamepad1.y) {
                if (intakeServoPosition != 0.6) {
                    intakeServo.setPosition(0.6);
                    intakeServoPosition = 0.6;
                }
                intakeMotor.setPower(0);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
            } else if (!forceIntake){
                intakeMotor.setPower(0);
            }

            if(currentTime - lastUpdateTime > 50) lastUpdateTime = System.currentTimeMillis();

        }

        if(!turretRotated){
            if(lastAprilTagDetection.milliseconds() >= 50) {
                List<AprilTagDetection> detections = aprilTag.getFreshDetections();

                if (detections != null) {
                    for(AprilTagDetection detection : detections){
                        if(detection.id == targetGoalAprilTagID){
                            if(detection.ftcPose != null) {
                                if (Math.abs(detection.ftcPose.bearing) > 5.0) {
                                    currentServoPosition -= 0.05*detection.ftcPose.bearing/1800;
                                    currentServoPosition = Range.clip(currentServoPosition, 0.0, 0.4);
                                    rightServo.setPosition(currentServoPosition);
                                    leftServo.setPosition(currentServoPosition);
                                }
                            }
                        }
                    }
                    lastAprilTagDetection.reset();
                }
            }
        }

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, blobs);

        drive.update();

        lastCall.reset();
    }
}
