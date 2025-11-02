package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="PrajwalOpMode")
public class PrajwalOpMode extends OpMode {
    private DcMotor[] motors = new DcMotor[4];
    private DcMotor intakeMotor;

    private Servo intakeServo;

    private double diag1, diag2, fl, bl, fr, br, max, leftX, rightX, leftY/*, rightY*/, intakeServoPosition;
    private long lastUpdateTime, lastIntakeTime;

    SampleMecanumDrive drive;
    Trajectory trajectory;

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
        intakeServo.setPosition(0);

        for(int i = 0; i < motors.length; i+=1) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection((i >= 2) ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            //motors[i].setDirection(DcMotor.Direction.FORWARD);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(12)
                .build();
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void loop(){
        drive.update();

        //drivetrain
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        leftY = gamepad1.left_stick_y;

        diag1 = leftY + leftX;
        diag2 = leftY - leftX;

        fl = diag1-rightX;
        bl = diag2-rightX;
        fr = diag2+rightX;
        br = diag1+rightX;

        max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
        if(max > 1){
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        motors[0].setPower(-fl);
        motors[1].setPower(-bl);
        motors[2].setPower(-fr);
        motors[3].setPower(-br);

        telemetry.addLine("-------DRIVETRAIN------");
        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl);
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);

        //intake
        long currentTime = System.currentTimeMillis();
        if (gamepad1.a) {
            intakeMotor.setPower(1.0);
            intakeServo.setPosition(1);
            intakeServoPosition = 1;
        } else if (gamepad1.y || currentTime - lastIntakeTime > 5000) {
            if(intakeServoPosition != 0) {
                intakeServo.setPosition(0);
                intakeServoPosition = 0;
            }
            intakeMotor.setPower(0);
        } else {
            intakeMotor.setPower(0);
            lastIntakeTime = currentTime;
        }

        telemetry.addLine("-------INTAKE------");
        telemetry.addData("servoPosition", intakeServoPosition);

        telemetry.update();
    }
}
