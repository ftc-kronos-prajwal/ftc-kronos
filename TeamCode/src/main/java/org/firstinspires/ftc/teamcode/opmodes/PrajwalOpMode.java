package org.firstinspires.ftc.teamcode.opmodes;

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
        intakeServo.setPosition(0.6);

        for(int i = 0; i < motors.length; i+=1) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setDirection((i >= 2) ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            //motors[i].setDirection(DcMotor.Direction.FORWARD);
        }

        /*drive = new SampleMecanumDrive(hardwareMap);

        //trajectory = drive.trajectoryBuilder(new Pose2d())
        //        .forward(12)
                .build();
        drive.followTrajectoryAsync(trajectory);*/
    }

    @Override
    public void loop(){
        //drive.update();

        //drivetrain
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;

        /*double angle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        if((angle > 3*Math.PI/8 && angle < 5*Math.PI/8) || (angle < -3*Math.PI/8 && angle > -5*Math.PI/8)){
            leftX = 0;
        }*/

        /*diag1 = leftY + leftX;
        diag2 = leftY - leftX;

        fl = leftY + leftX + rightX;
        bl = leftY - leftX + rightX;
        fr = leftY - leftX - rightX;
        br = leftY + leftX - rightX;

        max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
        if(max > 1){
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }*/

        double y = gamepad1.left_stick_y;  // Forward/backward (inverted)
        double x = gamepad1.left_stick_x;   // Strafe left/right
        double rx = -gamepad1.right_stick_x; // Rotation

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
        long currentTime = System.currentTimeMillis();
        if (gamepad1.a) {
            intakeMotor.setPower(-1.0);
            if(intakeServoPosition != 0.3){
                intakeServo.setPosition(0.3);
                intakeServoPosition = 0.3;
            }
        } else if (gamepad1.y/* || currentTime - lastIntakeTime > 1000*/) {
            if(intakeServoPosition != 0.6) {
                intakeServo.setPosition(0.6);
                intakeServoPosition = 0.6;
            }
            intakeMotor.setPower(0);
        } else if (gamepad1.x){
            intakeMotor.setPower(-1.0);
            if(currentTime - lastIntakeTime >= 250){
                intakeServo.setPosition(0);
                intakeServoPosition = 0;
            }
            if(intakeServoPosition != 0.3 && intakeServoPosition != 0) {
                lastIntakeTime = currentTime;
                intakeServo.setPosition(0.3);
                intakeServoPosition = 0.3;
            }
        }else{
            intakeMotor.setPower(0);
        }

        telemetry.addLine("-------INTAKE------");
        telemetry.addData("servoPosition", intakeServoPosition);

        telemetry.update();
    }
}
