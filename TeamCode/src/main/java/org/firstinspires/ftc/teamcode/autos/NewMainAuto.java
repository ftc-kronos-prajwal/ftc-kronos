package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class NewMainAuto extends OpMode {

    private SampleMecanumDrive drive;
    private double angle;

    private DcMotor intakeMotor, turretLaunchMotor;
    private Servo intakeServo;

    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start first trajectory when autonomous begins
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(4)
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void loop() {
        drive.update();
    }

}