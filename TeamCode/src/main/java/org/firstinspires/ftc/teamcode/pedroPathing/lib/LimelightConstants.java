package org.firstinspires.ftc.teamcode.pedroPathing.lib;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.os.Build;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * This is the LimelightConstants class. It holds many constants and parameters for the Limelight Localizer.
 * @author
 * @version
 */

@TargetApi(Build.VERSION_CODES.N)
public class LimelightConstants {

    /** The pipeline for the limelight
     * Default Value = 1
     */
    public int pipeline = 1;


    /** The name of the Limelight in the hardware map
     * Default Value: "limelight" */
    public  String hardwareMapName = "limelight";

    /** The maximum distance allowed in inches for single tag MT1 localization
     * Default Value: 60.0 (approximately 1.5m)
     */
    public double mt1MaxDistanceInches = 60;

    /** The maximum velocity allowed in inches/s for single tag MT1 localization
     * Default Value: 10.0 (approximately 0.25m/s)
     */
    public double mt1MaxVelocity = 10;

    /** The maximum skew allowed in degrees for single tag MT1 localization
     * Default Value: 10.0
     */
    public double mt1MaxSkewDegrees = 10;


    /** The final unaccounted latency (s) for limelight pose -> code
    * Default Value: 0.01 (10ms)
     */
    public double networkLatencySeconds = 0.01;


    /**
     * This creates a new LimelightConstants with default values.
     */
    public LimelightConstants() {
        defaults();
    }

    public LimelightConstants pipeline(int pipeline) {
        this.pipeline = pipeline;
        return this;
    }

    public LimelightConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public LimelightConstants mt1MaxDistanceInches(double mt1MaxDistanceInches) {
        this.mt1MaxDistanceInches = mt1MaxDistanceInches;
        return this;
    }

    public LimelightConstants mt1MaxVelocity(double mt1MaxVelocity) {
        this.mt1MaxVelocity = mt1MaxVelocity;
        return this;
    }

    public LimelightConstants mt1MaxSkewDegrees(double mt1MaxSkewDegrees) {
        this.mt1MaxSkewDegrees = mt1MaxSkewDegrees;
        return this;
    }

    public LimelightConstants networkLatencySeconds(double networkLatencySeconds) {
        this.networkLatencySeconds = networkLatencySeconds;
        return this;
    }

    public void defaults() {
        pipeline = 1;
        hardwareMapName = "limelight";
        mt1MaxDistanceInches = 60;
        mt1MaxVelocity = 10;
        mt1MaxSkewDegrees = 10;
        networkLatencySeconds = 0.01;
    }
}
