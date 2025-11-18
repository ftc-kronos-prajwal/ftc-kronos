package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class PrajwalAuto extends OpMode {
    private enum State {
        DRIVE_TO_SHOOT_1,
        SPIN_UP_SHOOTER_1,
        SHOOT_1,
        DRIVE_TO_PICKUP,
        WAIT_AFTER_DRIVE,
        INTAKE_ELEMENT,
        DRIVE_FORWARD,
        WAIT_AFTER_FORWARD,
        DRIVE_TO_SHOOT_2,
        WAIT_BEFORE_SHOOT_2,
        SPIN_UP_SHOOTER_2,
        SHOOT_2,
        DONE
    }

    private State currentState = State.DRIVE_TO_SHOOT_1;
    private long stateStartTime = 0;

    private SampleMecanumDrive drive;
    private double angle;

    private DcMotor intakeMotor, turretLaunchMotor;
    private Servo intakeServo;
    private CRServo leftServo, rightServo;

    @Override
    public void init(){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "servo");
        leftServo = hardwareMap.get(CRServo.class, "leftservo");
        rightServo = hardwareMap.get(CRServo.class, "rightservo");
        turretLaunchMotor = hardwareMap.get(DcMotor.class,"shootermotor");

        turretLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        drive = new SampleMecanumDrive(hardwareMap);

        angle = Math.atan2(58-24, -62+52);

        Pose2d startPose = new Pose2d(0, 24, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        intakeServo.setPosition(0.3);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start first trajectory when autonomous begins
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0, 24, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-52, 24, angle))
                .build();
        drive.followTrajectorySequenceAsync(traj);
        stateStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop(){
        // Emergency stop
        if(gamepad1.a){
            requestOpModeStop();
            return;
        }

        drive.update();

        // Display current state
        telemetry.addData("State", currentState);
        telemetry.addData("Time in state", (System.currentTimeMillis() - stateStartTime) / 1000.0);
        telemetry.update();

        switch(currentState) {
            case DRIVE_TO_SHOOT_1:
                if(!drive.isBusy()) {
                    transitionTo(State.SPIN_UP_SHOOTER_1);
                    turretLaunchMotor.setPower(-1.0);
                }
                break;

            case SPIN_UP_SHOOTER_1:
                if(timeInState() >= 2000) {
                    transitionTo(State.SHOOT_1);
                    intakeMotor.setPower(-1.0);
                }
                break;

            case SHOOT_1:
                if(timeInState() >= 2000) {
                    stopShooter();

                    TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(90)))
                            .build();
                    drive.followTrajectorySequenceAsync(traj);

                    transitionTo(State.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:
                if(!drive.isBusy()) {
                    transitionTo(State.WAIT_AFTER_DRIVE);
                }
                break;

            case WAIT_AFTER_DRIVE:
                if(timeInState() >= 500) {
                    transitionTo(State.INTAKE_ELEMENT);
                    intakeServo.setPosition(0.3);
                    intakeMotor.setPower(-1.0);
                }
                break;

            case INTAKE_ELEMENT:
                if(timeInState() >= 500) {
                    intakeMotor.setPower(0);

                    TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .forward(18)
                            .build();
                    drive.followTrajectorySequenceAsync(traj);

                    transitionTo(State.DRIVE_FORWARD);
                }
                break;

            case DRIVE_FORWARD:
                if(!drive.isBusy()) {
                    transitionTo(State.WAIT_AFTER_FORWARD);
                }
                break;

            case WAIT_AFTER_FORWARD:
                if(timeInState() >= 500) {
                    intakeServo.setPosition(0.3);
                    intakeMotor.setPower(0.0);

                    TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-52, 24, angle))
                            .build();
                    drive.followTrajectorySequenceAsync(traj);

                    transitionTo(State.DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if(!drive.isBusy()) {
                    transitionTo(State.WAIT_BEFORE_SHOOT_2);
                }
                break;

            case WAIT_BEFORE_SHOOT_2:
                if(timeInState() >= 500) {
                    transitionTo(State.SPIN_UP_SHOOTER_2);
                    turretLaunchMotor.setPower(-1.0);
                }
                break;

            case SPIN_UP_SHOOTER_2:
                if(timeInState() >= 2000) {
                    transitionTo(State.SHOOT_2);
                    intakeMotor.setPower(-1.0);
                }
                break;

            case SHOOT_2:
                if(timeInState() >= 2000) {
                    stopShooter();
                    transitionTo(State.DONE);
                    requestOpModeStop();
                }
                break;

            case DONE:
                // Do nothing, OpMode will stop
                break;
        }
    }

    private void transitionTo(State newState) {
        currentState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private long timeInState() {
        return System.currentTimeMillis() - stateStartTime;
    }

    private void stopShooter() {
        intakeMotor.setPower(0.0);
        intakeServo.setPosition(0.3);
        turretLaunchMotor.setPower(0.0);
    }
}