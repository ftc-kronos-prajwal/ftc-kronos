package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private DcMotor intakeMotor, turretLaunchMotor;

    private Servo intakeServo;

    private double diag1, diag2, fl, bl, fr, br, max, leftX, rightX, leftY/*, rightY*/, intakeServoPosition, turretRotPower = 0, turretLaunchPower = 0;
    private long lastUpdateTime, lastIntakeTime, lastTurretRotTime, currentTime;
    private boolean lastTurretRotLogged = false, pLBState = false, pRBState = false;

    private short lastTurretRotDir = 0;
    private CRServo leftServo, rightServo;

    SampleMecanumDrive drive;
    TrajectorySequence trajectory;

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
        intakeServo.setPosition(0.6);

        for(int i = 0; i < motors.length; i+=1) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //motors[i].setDirection((i >= 2) ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            //motors[i].setDirection(DcMotor.Direction.FORWARD);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(0, 0, 0);
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());

        drive.setPoseEstimate(pose);
        trajectory = drive.trajectorySequenceBuilder(pose)
                .lineTo(new Vector2d(-6.0, 6.0))
                .turn(Math.toRadians(90))
                .build();
        //drive.followTrajectorySequenceAsync(trajectory);

        leftServo = hardwareMap.get(CRServo.class, "leftservo");
        rightServo = hardwareMap.get(CRServo.class, "rightservo");
        turretLaunchMotor = hardwareMap.get(DcMotor.class,"shootermotor");
    }

    @Override
    public void loop(){
        //drive.update();

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

            leftServo.setPower(turretRotPower);
            rightServo.setPower(turretRotPower);

            if(currentTime - lastUpdateTime > 50) {
                double prev = turretLaunchPower;

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
            }

            turretLaunchMotor.setPower(turretLaunchPower);

            //drivetrain

            double y = -gamepad1.left_stick_y;  // Forward/backward (inverted)
            double x = gamepad1.left_stick_x;   // Strafe left/right
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

            //intake
            currentTime = System.currentTimeMillis();
            if (gamepad1.a) {
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
            } else {
                intakeMotor.setPower(0);
            }

            if(currentTime - lastUpdateTime > 50) lastUpdateTime = System.currentTimeMillis();
        }
    }
}
