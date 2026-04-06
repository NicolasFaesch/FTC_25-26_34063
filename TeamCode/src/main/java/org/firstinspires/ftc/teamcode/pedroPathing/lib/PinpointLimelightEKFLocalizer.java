package org.firstinspires.ftc.teamcode.pedroPathing.lib;

import android.annotation.SuppressLint;

import com.pedropathing.localization.Localizer;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;

/**
 * Pinpoint Limelight EKF Localizer
 * Fully integrated localizer that feeds Pinpoint IMU data to the Limelight
 * for Megatag 2, and runs a latency-compensated Complementary Filter.
 */
public class PinpointLimelightEKFLocalizer implements Localizer {
    private final GoBildaPinpointDriver odo;
    private final Limelight3A limelight;

    private final PinpointConstants pinpointConstants;
    private final LimelightConstants limelightConstants;
    private final EKFConstants ekfConstants;

    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private Pose currentVelocity;
    private LLResult lastVisionResult = null;
    private boolean hasGlobalHeadingFix = false;

    // Tracking poses
    private Pose pinpointPose;
    private Pose fusedPose;

    // Latency Compensation Buffer State Object
    private static class OdomState {
        Pose pose;
        Pose velocity;

        public OdomState(Pose pose, Pose velocity) {
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    // Latency Compensation Buffer
    private final TreeMap<Double, OdomState> odomHistory = new TreeMap<>();
    private static final double MAX_HISTORY_SECONDS = 1.0;

    public PinpointLimelightEKFLocalizer(HardwareMap map, PinpointConstants pinpointConstants, LimelightConstants limelightConstants, EKFConstants ekfConstants) {
        this(map, pinpointConstants, limelightConstants, ekfConstants, new Pose());
    }

    @SuppressLint("NewApi")
    public PinpointLimelightEKFLocalizer(HardwareMap map, PinpointConstants pinpointConstants, LimelightConstants limelightConstants, EKFConstants ekfConstants, Pose setStartPose) {
        // Initialize Pinpoint
        odo = map.get(GoBildaPinpointDriver.class, pinpointConstants.hardwareMapName);
        setOffsets(pinpointConstants.forwardPodY, pinpointConstants.strafePodX, pinpointConstants.distanceUnit);

        if (pinpointConstants.yawScalar.isPresent()) {
            odo.setYawScalar(pinpointConstants.yawScalar.getAsDouble());
        }
        if (pinpointConstants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(pinpointConstants.customEncoderResolution.getAsDouble(), pinpointConstants.distanceUnit);
        } else {
            odo.setEncoderResolution(pinpointConstants.encoderResolution);
        }
        odo.setEncoderDirections(pinpointConstants.forwardEncoderDirection, pinpointConstants.strafeEncoderDirection);

        // Initialize Limelight
        limelight = map.get(Limelight3A.class, limelightConstants.hardwareMapName);
        limelight.pipelineSwitch(limelightConstants.pipeline); // Default pipeline, adjust if necessary
        limelight.start();

        this.pinpointConstants = pinpointConstants;
        this.limelightConstants = limelightConstants;
        this.ekfConstants = ekfConstants;

        setStartPose(setStartPose);
        totalHeading = 0;

        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();
    }

    @Override
    public Pose getPose() {
        return fusedPose; // Return the fused estimate to PedroPathing
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = pinpointPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }
        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(PoseConverter.poseToPose2D(setPose, PedroCoordinates.INSTANCE));
        pinpointPose = setPose;
        fusedPose = setPose;
        previousHeading = setPose.getHeading();
        odomHistory.clear();
        hasGlobalHeadingFix = true; // Lock in the heading since we are declaring a trusted pose
    }

    /**
     * The Main Loop. Handles Odometry Tracking and Vision Polling automatically.
     */
    @Override
    public void update() {
        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        Pose currentPinpointVelocity = PoseConverter.pose2DToPose(
                new Pose2D(DistanceUnit.MM, odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), AngleUnit.RADIANS, odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
                , PedroCoordinates.INSTANCE);

        // --- 1. ODOMETRY PREDICT STEP ---
        double globalDx = currentPinpointPose.getX() - pinpointPose.getX();
        double globalDy = currentPinpointPose.getY() - pinpointPose.getY();
        double deltaHeading = angleWrap(currentPinpointPose.getHeading() - pinpointPose.getHeading());

        // Calculate the drift between the Pinpoint's grid and our True Fused grid.
        double headingDesync = angleWrap(fusedPose.getHeading() - pinpointPose.getHeading());

        // Rotate the Pinpoint's deltas to match the Fused global coordinate frame
        double fusedDx = (globalDx * Math.cos(headingDesync)) - (globalDy * Math.sin(headingDesync));
        double fusedDy = (globalDx * Math.sin(headingDesync)) + (globalDy * Math.cos(headingDesync));

        fusedPose = new Pose(
                fusedPose.getX() + fusedDx,
                fusedPose.getY() + fusedDy,
                angleWrap(fusedPose.getHeading() + deltaHeading)
        );

        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        previousHeading = currentPinpointPose.getHeading();

        // Rotate the Pinpoint's velocity vector to match the Fused global coordinate frame
        double vX = currentPinpointVelocity.getX();
        double vY = currentPinpointVelocity.getY();

        double fusedVx = (vX * Math.cos(headingDesync)) - (vY * Math.sin(headingDesync));
        double fusedVy = (vX * Math.sin(headingDesync)) + (vY * Math.cos(headingDesync));

        // currentPinpointVelocity.getHeading() represents the angular velocity here
        currentVelocity = new Pose(fusedVx, fusedVy, currentPinpointVelocity.getHeading());

        pinpointPose = currentPinpointPose;

        // Log history for latency compensation (Pose AND Velocity)
        double currentTimeSec = System.nanoTime() / 1.0E9;
        odomHistory.put(currentTimeSec, new OdomState(pinpointPose, currentVelocity));
        odomHistory.headMap(currentTimeSec - MAX_HISTORY_SECONDS).clear();

        // --- VISION UPDATE STEP ---
        // Feed the IMU heading to the Limelight for MT2 stability (LL expects degrees)
        limelight.updateRobotOrientation(Math.toDegrees(fusedPose.getHeading()));

        LLResult result = limelight.getLatestResult();
        // Check if valid and new result
        if (result != null && result.isValid() && result != lastVisionResult) {
            processLimelightData(result, currentTimeSec);
            lastVisionResult = result; // Save this frame so we don't process it again
        }
    }

    /**
     * Extracts data from the LLResult and feeds it into the fusion filter.
     */
    private void processLimelightData(LLResult result, double currentTimeSec) {
        // Convert latency to seconds
        double captureLatencySec = result.getCaptureLatency() / 1000.0;
        double targetingLatencySec = result.getTargetingLatency() / 1000.0;

        // Convert distance (m) to inches
        double tagDistanceInches = result.getBotposeAvgDist() / 0.0254;
        int tagCount = result.getFiducialResults().size();

        // Current speed is used only for routing logic, not EKF penalties
        double currentRobotSpeed = Math.hypot(currentVelocity.getX(), currentVelocity.getY());

        // Extract the viewing angle (skew) of the tag
        double tagSkew = 0.0;
        if (tagCount > 0) {
            Pose3D targetPose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
            if (targetPose != null) {
                // The yaw of the tag in camera space represents the horizontal viewing angle
                tagSkew = Math.abs(targetPose.getOrientation().getYaw());
            }
        }

        // Smart MT1 vs MT2 Routing Logic with Skew Thresholds
        boolean useMT1 = false;

        if (!hasGlobalHeadingFix) {
            // We NEED MT1 to fix our heading, but we refuse to use it if the skew is too high
            if (tagCount > 0 && tagDistanceInches < limelightConstants.mt1MaxDistanceInches && tagSkew < limelightConstants.mt1MaxSkewDegrees) {
                useMT1 = true;
                hasGlobalHeadingFix = true; // We got a solid, straight-on reading! Unlock MT2 for the future.
            } else {
                return; // Too far away or too skewed. Trust pure odometry until we drive closer/straighter.
            }
        } else if (tagCount > 1) {
            useMT1 = true; // Multiple tags practically eliminate ambiguity, skew doesn't matter much
        } else if (tagDistanceInches < limelightConstants.mt1MaxDistanceInches
                && currentRobotSpeed < limelightConstants.mt1MaxVelocity
                && tagSkew < limelightConstants.mt1MaxSkewDegrees) {
            useMT1 = true; // Close enough, slow enough, and straight-on enough to trust MT1 heading
        }

        Pose visionPose;
        if (useMT1) {
            Pose3D mt1Pose = result.getBotpose();
            visionPose = new Pose(
                    mt1Pose.getPosition().x / 0.0254,
                    mt1Pose.getPosition().y / 0.0254,
                    Math.toRadians(mt1Pose.getOrientation().getYaw())
            );
        } else {
            Pose3D mt2Pose = result.getBotpose_MT2();
            visionPose = new Pose(
                    mt2Pose.getPosition().x / 0.0254,
                    mt2Pose.getPosition().y / 0.0254,
                    Math.toRadians(mt2Pose.getOrientation().getYaw())
            );
        }

        // Ambiguity from number of tags. If we see two, lower impact of distance on noise.
        double ambiguity = (tagCount > 1) ? ekfConstants.dualTagDistanceScaler : 1.0;

        // Run the Latency-Compensated Filter, passing in our new tagSkew
        updateFusion(visionPose, captureLatencySec, targetingLatencySec, currentTimeSec, !useMT1, tagDistanceInches, ambiguity, tagSkew);
    }

    /**
     * Latency compensation and weighting step.
     */
    private void updateFusion(Pose visionPose, double captureLatencySec, double pipelineLatencySec,
                              double currentTimeSec, boolean isMT2, double tagDistance,
                              double ambiguity, double tagSkew) {

        double totalLatency = captureLatencySec + pipelineLatencySec + limelightConstants.networkLatencySeconds;
        double imageTimestamp = currentTimeSec - totalLatency;

        // Grab bounding poses for interpolation
        Map.Entry<Double, OdomState> floorEntry = odomHistory.floorEntry(imageTimestamp);
        Map.Entry<Double, OdomState> ceilingEntry = odomHistory.ceilingEntry(imageTimestamp);

        if (floorEntry == null) return; // History cleared or too old

        OdomState pastState;

        // Linearly interpolate the pose and velocity if we have brackets
        if (ceilingEntry != null && !floorEntry.getKey().equals(ceilingEntry.getKey())) {
            double t0 = floorEntry.getKey();
            double t1 = ceilingEntry.getKey();
            double t = (imageTimestamp - t0) / (t1 - t0); // Percentage between the two frames

            Pose p0 = floorEntry.getValue().pose;
            Pose p1 = ceilingEntry.getValue().pose;

            // Interpolate Pose
            double interpX = p0.getX() + (p1.getX() - p0.getX()) * t;
            double interpY = p0.getY() + (p1.getY() - p0.getY()) * t;
            double interpHeading = angleWrap(p0.getHeading() + angleWrap(p1.getHeading() - p0.getHeading()) * t);
            Pose pastOdomPose = new Pose(interpX, interpY, interpHeading);

            // Interpolate Velocity (crucial for accurate noise penalty)
            Pose v0 = floorEntry.getValue().velocity;
            Pose v1 = ceilingEntry.getValue().velocity;
            Pose pastVelocity = new Pose(
                    v0.getX() + (v1.getX() - v0.getX()) * t,
                    v0.getY() + (v1.getY() - v0.getY()) * t,
                    v0.getHeading() + (v1.getHeading() - v0.getHeading()) * t
            );

            pastState = new OdomState(pastOdomPose, pastVelocity);
        } else {
            pastState = floorEntry.getValue(); // Fallback if no ceiling
        }

        Pose pastOdomPose = pastState.pose;

        // Calculate historical speeds for the EKF penalty
        double historicalRobotSpeed = Math.hypot(pastState.velocity.getX(), pastState.velocity.getY());
        double historicalAngularSpeed = Math.abs(pastState.velocity.getHeading());

        // Calculate Odometry Delta (movement in the Pinpoint's coordinate frame DURING the delay)
        double pinpointDx = pinpointPose.getX() - pastOdomPose.getX();
        double pinpointDy = pinpointPose.getY() - pastOdomPose.getY();
        double deltaHeading = angleWrap(pinpointPose.getHeading() - pastOdomPose.getHeading());

        // Calculate the difference between the Vision's true heading and the Pinpoint's historical heading
        double headingDesync = angleWrap(visionPose.getHeading() - pastOdomPose.getHeading());

        // Rotate the Pinpoint's movement vector to align with the global Vision coordinate frame
        double adjustedDx = (pinpointDx * Math.cos(headingDesync)) - (pinpointDy * Math.sin(headingDesync));
        double adjustedDy = (pinpointDx * Math.sin(headingDesync)) + (pinpointDy * Math.cos(headingDesync));

        // Fast-forward vision pose to the present
        Pose adjustedVisionPose = new Pose(
                visionPose.getX() + adjustedDx,
                visionPose.getY() + adjustedDy,
                angleWrap(visionPose.getHeading() + deltaHeading)
        );

        // Dynamic Confidence (Covariance) Calculation using HISTORICAL speeds and SKEW
        double visionNoise = ekfConstants.noiseBase
                + (ekfConstants.kVelocity * Math.pow(historicalRobotSpeed, 2))
                + (ekfConstants.kAngularVelocity * Math.pow(Math.toDegrees(historicalAngularSpeed), 3))
                + (ekfConstants.kDistance * Math.pow(ambiguity * tagDistance, ekfConstants.distanceExponential))
                + (ekfConstants.kSkew * Math.pow(tagSkew, 2)); // Quadratic penalty for viewing angle

        double visionWeight = ekfConstants.visionWeightScaler * 1.0 / (1.0 + visionNoise);

        applyFusion(adjustedVisionPose, visionWeight, isMT2);
    }

    /**
     * Blends the latency-adjusted vision pose with the current fused estimate.
     */
    private void applyFusion(Pose adjustedVisionPose, double visionWeight, boolean isMT2) {
        double newX = (fusedPose.getX() * (1 - visionWeight)) + (adjustedVisionPose.getX() * visionWeight);
        double newY = (fusedPose.getY() * (1 - visionWeight)) + (adjustedVisionPose.getY() * visionWeight);
        double newHeading = fusedPose.getHeading();

        if (!isMT2) { // Only update heading if we used Megatag 1
            double headingDiff = angleWrap(adjustedVisionPose.getHeading() - fusedPose.getHeading());
            newHeading = angleWrap(fusedPose.getHeading() + (headingDiff * visionWeight));
        }

        fusedPose = new Pose(newX, newY, newHeading);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    // --- Standard PedroPathing Getters and Setters ---

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    @Override
    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    @Override
    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(xOffset, yOffset, unit);
    }

    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    private void resetPinpoint() {
        odo.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void recalibrate() {
        odo.recalibrateIMU();
    }

    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    @Override
    public void setX(double x) {
        odo.setPosX(x, pinpointConstants.distanceUnit);
        fusedPose = new Pose(x, fusedPose.getY(), fusedPose.getHeading());
        odomHistory.clear();
    }

    @Override
    public void setY(double y) {
        odo.setPosY(y, pinpointConstants.distanceUnit);
        fusedPose = new Pose(fusedPose.getX(), y, fusedPose.getHeading());
        odomHistory.clear();
    }

    @Override
    public void setHeading(double heading) {
        odo.setHeading(heading, AngleUnit.RADIANS);
        fusedPose = new Pose(fusedPose.getX(), fusedPose.getY(), heading);
        odomHistory.clear();
    }
}