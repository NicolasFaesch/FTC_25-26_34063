package org.firstinspires.ftc.teamcode.pedroPathing.lib;

import android.annotation.TargetApi;
import android.os.Build;
import android.annotation.SuppressLint;

import java.util.OptionalDouble;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/**
 * This is the EKFConstants class. It holds many constants and parameters for the Extended Kalman Filter Localizer that fuses limelight with pinpoint odometry.
 * @author
 * @version
 */

@TargetApi(Build.VERSION_CODES.N)
public class EKFConstants {

    /** This multiplier scales the vision weight in the fused pose estimate.
     *  Default Value: 0.2
     */
    public double visionWeightScaler = 0.2;

    /** This is your baseline distrust of the camera under absolutely perfect conditions
     * Default Value: 0.5
     */
    public double noiseBase = 0.5;

    /** This scales the noise with robot velocity.
     * Default Value: 0.05
     */
    public double kVelocity = 0.05;

    /** This scales the noise with robot angular velocity (deg/s).
     * Default Value: 0.00001
     */
    public double kAngularVelocity = 0.00001;

    /** This scales the noise with tag distance.
     * Default Value: 0.1
     */
    public double kDistance = 0.02;

    /** Penalty of the angle towards single tag detection.
     * Default Value: 0.01
     */
    public double kSkew = 0.01;

    /** This value is the exponential that is applied on the tagDistance. (HAS to be >1)
     * Default Value: 1.3
     */
    public double distanceExponential = 1.3;

    /** This scalar reduces the impact of distance on the noise in the case of multi-tag detection.
     * Default Value: 0.4
     */
    public double dualTagDistanceScaler = 0.4;


    /**
     * This creates a new EKFConstants with default values.
     */
    public EKFConstants() {
        defaults();
    }

    public EKFConstants visionWeightScaler(double visionWeightScaler) {
        this.visionWeightScaler = visionWeightScaler;
        return this;
    }

    public EKFConstants noiseBase(double noiseBase) {
        this.noiseBase = noiseBase;
        return this;
    }

    public EKFConstants kVelocity(double kVelocity) {
        this.kVelocity = kVelocity;
        return this;
    }

    public EKFConstants kAngularVelocity(double kAngularVelocity) {
        this.kAngularVelocity = kAngularVelocity;
        return this;
    }

    public EKFConstants kDistance(double kDistance) {
        this.kDistance = kDistance;
        return this;
    }

    public EKFConstants kSkew(double kSkew) {
        this.kSkew = kSkew;
        return this;
    }

    public EKFConstants distanceExponential(double distanceExponential) {
        this.distanceExponential = distanceExponential;
        return this;
    }

    public EKFConstants dualTagDistanceScaler(double dualTagDistanceScaler) {
        this.dualTagDistanceScaler = dualTagDistanceScaler;
        return this;
    }



    public void defaults() {
        visionWeightScaler = 0.2;
        noiseBase = 0.5;
        kVelocity = 0.05;
        kAngularVelocity = 0.00001;
        kDistance = 0.02;
        kSkew = 0.01;
        distanceExponential = 1.3;
        dualTagDistanceScaler = 0.4;
    }
}
