package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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

    private NormalizedColorSensor colorSensor;

    private SampleMecanumDrive drive;
    private TrajectorySequence trajectory;

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
        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(0, 0, 0);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());

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

            leftServo.setPower(gamepad1.left_bumper ? -1.0 : (gamepad1.right_bumper ? 1.0 : 0.0));
            rightServo.setPower(gamepad1.left_bumper ? -1.0 : (gamepad1.right_bumper ? 1.0 : 0.0));

            turretLaunchMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            if(Math.abs(gamepad1.right_trigger-gamepad1.left_trigger) >= 0.95){
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
            } else if (!forceIntake){
                intakeMotor.setPower(0);
            }

            if(currentTime - lastUpdateTime > 50) lastUpdateTime = System.currentTimeMillis();

        }
        drive.update();
    }
}
