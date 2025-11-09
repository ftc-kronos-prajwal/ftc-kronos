package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -7;
    public static double PARALLEL_Y = -1.5;

    public static double PERPENDICULAR_X = -7;
    public static double PERPENDICULAR_Y = 1.5;

    public static double X_MULTIPLIER = 72/71.6295;
    public static double Y_MULTIPLIER = 72/72.045;

    private Encoder parallelEncoder, perpendicularEncoder;
    private SampleMecanumDrive drive;

    // --- Smoothing variables ---
    private double lastVParallel = 0;
    private double lastVPerp = 0;

    // lower = smoother. 0.2 works well for most bots
    private static final double ALPHA = 0.2;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));

        // If one wheel reports negative distance, reverse it here
        // parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        double rawParallel = encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER;
        double rawPerp = encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER;

        lastVParallel = ALPHA * rawParallel + (1 - ALPHA) * lastVParallel;
        lastVPerp     = ALPHA * rawPerp     + (1 - ALPHA) * lastVPerp;

        return Arrays.asList(lastVParallel, lastVPerp);
    }
}
