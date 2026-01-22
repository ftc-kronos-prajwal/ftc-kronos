package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double angle = Math.toRadians(135);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(43.32, 43.32, 4.8, Math.toRadians(180), 14.52)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-63, 24, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-36, 36, angle))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-12, 32, Math.toRadians(90)))
                        .forward(18)
                        .lineToLinearHeading(new Pose2d(-36, 36, angle))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(90)))
                        .forward(18)
                        .lineToLinearHeading(new Pose2d(-36, 36, angle))
                        .waitSeconds(1)
                        .build());

        Image img = null;
        try{ img = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/images/field-2025-juice-dark.png")); }
        catch(Exception e){ e.printStackTrace(); }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}