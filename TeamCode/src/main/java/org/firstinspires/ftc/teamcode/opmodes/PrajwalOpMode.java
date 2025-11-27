package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private DcMotor intakeMotor, turretLaunchMotor, kickerMotor;

    private Servo intakeServo, stopperServo;

    private double diag1, diag2, fl, bl, fr, br, max, leftX, rightX, leftY/*, rightY*/, intakeServoPosition = 0.6, stopperServoPosition = 0.8, turretRotPower = 0, turretLaunchPower = 0, kickerMotorPower = 0.0;
    private long lastUpdateTime, lastIntakeTime, lastTurretRotTime, currentTime;
    private boolean lastTurretRotLogged = false, pLBState = false, pRBState = false;

    private short lastTurretRotDir = 0;
    private CRServo leftServo, rightServo;

    private ElapsedTime timer = new ElapsedTime();

    NormalizedColorSensor colorSensor;

    SampleMecanumDrive drive;
    TrajectorySequence trajectory;

    boolean forceIntake = false;

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
            //motors[i].setDirection((i >= 2) ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            //motors[i].setDirection(DcMotor.Direction.FORWARD);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(0, 0, 0);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());

        //drive.setPoseEstimate(pose);
        /*trajectory = drive.trajectorySequenceBuilder(pose)
                .lineTo(new Vector2d(-6.0, 6.0))
                .turn(Math.toRadians(90))
                .build();*/
        //drive.followTrajectorySequenceAsync(trajectory);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftServo = hardwareMap.get(CRServo.class, "leftservo");
        rightServo = hardwareMap.get(CRServo.class, "rightservo");
        turretLaunchMotor = hardwareMap.get(DcMotor.class,"shootermotor");

        turretLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    @Override
    public void loop(){
        //drivetrain

        Pose2d estimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-estimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
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
            if(currentTime - lastUpdateTime > 50) {
                if (gamepad1.left_bumper) {
                    if (lastTurretRotDir != 1) {
                        turretRotPower = 0;
                        lastTurretRotDir = 1;
                        lastTurretRotLogged = false;
                    }
                    if (!pLBState && turretRotPower != 0) {
                        turretRotPower += 0.5;
                        if (turretRotPower > 1.0) turretRotPower = 1.0;
                        if (turretRotPower < -1.0) turretRotPower = -1.0;
                        pLBState = true;
                        pRBState = false;
                    }
                    if (lastTurretRotLogged) {
                        turretRotPower += ((double) (currentTime - lastTurretRotTime)) / 500.0;
                        if (turretRotPower > 1.0) turretRotPower = 1.0;
                        if (turretRotPower < -1.0) turretRotPower = -1.0;
                    }
                    lastTurretRotTime = currentTime;
                    lastTurretRotLogged = true;
                } else if (gamepad1.right_bumper) {
                    if (lastTurretRotDir != -1) {
                        turretRotPower = 0;
                        lastTurretRotDir = -1;
                        lastTurretRotLogged = false;
                    }
                    if (!pRBState && turretRotPower != 0) {
                        turretRotPower -= 0.5;
                        if (turretRotPower > 1.0) turretRotPower = 1.0;
                        if (turretRotPower < -1.0) turretRotPower = -1.0;
                        pLBState = false;
                        pRBState = true;
                    }
                    if (lastTurretRotLogged) {
                        turretRotPower -= ((double) (currentTime - lastTurretRotTime)) / 500.0;
                        if (turretRotPower > 1.0) turretRotPower = 1.0;
                        if (turretRotPower < -1.0) turretRotPower = -1.0;
                    }
                    lastTurretRotTime = currentTime;
                    lastTurretRotLogged = true;
                } else {
                    turretRotPower = 0;
                    lastTurretRotDir = 0;
                    lastTurretRotLogged = false;
                    pLBState = false;
                    pRBState = false;
                }
            }

            leftServo.setPower(-turretRotPower);
            rightServo.setPower(-turretRotPower);

            /*if(currentTime - lastUpdateTime > 50) {
                double prev = turretLaunchPower;
e
                if (gamepad1.right_trigger > 0) {
                    if (turretLaunchPower <= 0) {
                        turretLaunchPower = 0.1;
                    }
                    turretLaunchPower *= (gamepad1.right_trigger + 1);
                    if (turretLaunchPower > prev + gamepad1.right_trigger / 10) {
                        turretLaunchPower = prev + gamepad1.right_trigger / 10;
                    }
                    if (turretLaunchPower > gamepad1.right_trigger) {
                        turretLaunchPower = gamepad1.right_trigger;
                    }
                } else if (gamepad1.left_trigger > 0) {
                    if (turretLaunchPower >= 0) {
                        turretLaunchPower = -0.1;
                    }
                    turretLaunchPower *= (gamepad1.left_trigger + 1);
                    if (turretLaunchPower < prev - gamepad1.left_trigger / 10) {
                        turretLaunchPower = prev - gamepad1.left_trigger / 10;
                    }
                    if (turretLaunchPower < -gamepad1.left_trigger) {
                        turretLaunchPower = -gamepad1.left_trigger;
                    }
                }else{
                    turretLaunchPower = 0;
                }

                if (turretLaunchPower > 1.0) turretLaunchPower = 1.0;
                if (turretLaunchPower < -1.0) turretLaunchPower = -1.0;
            }*/
            turretLaunchMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            if(Math.abs(gamepad1.right_trigger-gamepad1.left_trigger) >= 0.95){
                if(timer.milliseconds() > 2000){
                    if(stopperServoPosition != 0.7){
                        stopperServo.setPosition(0.7);
                        stopperServoPosition = 0.7;
                    }
                    intakeMotor.setPower(0.0);
                }else if(timer.milliseconds() > 1000){
                    if(stopperServoPosition != 0.85){
                        stopperServo.setPosition(0.85);
                        stopperServoPosition = 0.85;
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
                if(stopperServoPosition != 0.85){
                    stopperServo.setPosition(0.85);
                    stopperServoPosition = 0.85;
                    intakeMotor.setPower(0.0);
                }
                if(kickerMotorPower != 0.0){
                    kickerMotor.setPower(0.0);
                    kickerMotorPower = 0.0;
                }
                forceIntake = false;
            }
            telemetry.addData("Turret Power", turretLaunchPower);
            NormalizedRGBA color = colorSensor.getNormalizedColors();
            telemetry.addData("Color r", color.red);
            telemetry.addData("Color g", color.green);
            telemetry.addData("Color b", color.blue);
            telemetry.addData("Color a", color.alpha);
            telemetry.update();

            //turretLaunchMotor.setPower(turretLaunchPower);

            //drivetrain

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
                if(stopperServoPosition != 0.85){
                    stopperServo.setPosition(0.85);
                    stopperServoPosition = 0.85;
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
            } else if (gamepad1.y/* || currentTime - lastIntakeTime > 1000*/) {
                if (intakeServoPosition != 0.6) {
                    intakeServo.setPosition(0.6);
                    intakeServoPosition = 0.6;
                }
                intakeMotor.setPower(0);
            } else if (gamepad1.x) {
                intakeMotor.setPower(-1.0);
                if (currentTime - lastIntakeTime >= 250) {
                    intakeServo.setPosition(0);
                    intakeServoPosition = 0;
                }
                if (intakeServoPosition != 0.3 && intakeServoPosition != 0) {
                    lastIntakeTime = currentTime;
                    intakeServo.setPosition(0.3);
                    intakeServoPosition = 0.3;
                }
            } else if (gamepad1.b){
                intakeMotor.setPower(-1.0);
                if (intakeServoPosition != 0.3) {
                    intakeServo.setPosition(0.3);
                    intakeServoPosition = 0.3;
                }
            } else if (!forceIntake){
                intakeMotor.setPower(0);
            }

            if(currentTime - lastUpdateTime > 50) lastUpdateTime = System.currentTimeMillis();

        }
        drive.update();
    }
}
