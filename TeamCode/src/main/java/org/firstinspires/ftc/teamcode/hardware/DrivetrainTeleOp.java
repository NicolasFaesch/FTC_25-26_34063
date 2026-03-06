package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

public class DrivetrainTeleOp extends Drivetrain{
    private static final double FORWARD_SCALING = 1.0;
    private static final double STRAFE_SCALING = 1.0;
    private static final double TURN_SCALING = 1.0;


    private static final double FORWARD_SCALING_SLOW = 0.3;
    private static final double STRAFE_SCALING_SLOW = 0.3;
    private static final double TURN_SCALING_SLOW = 0.3;

    // auto-aim PID gains
    private static final double AUTO_AIM_KP = 0.025;
    private static final double AUTO_AIM_KD = 0.0005;

    private boolean aimingMode;

    private boolean slowMode;

    private boolean parkingMode;


    private double manualAdjustmentFar = 0;
    private double manualAdjustmentNear = 0;


    private double previousHeadingError = 0;


    public enum goalDistance {
        CLOSE,
        FAR
    };

    private goalDistance currentDistance;

    public DrivetrainTeleOp(HardwareMap hardwareMap, Pose2D startPose) {
        super(hardwareMap, startPose);
        follower.startTeleOpDrive(false);

        aimingMode = false;
        slowMode = false;
        parkingMode = false;
    }

    public void update(double leftStickX, double leftStickY, double rightStickX, double loopTime,Pose2D robotPose) {
        if(parkingMode) {
            Pose2D botpose = getPose();

            //Gets the Heading Error (probably (not))
            double headingError = PoseStorage.parkingHeading - botpose.getHeading(AngleUnit.DEGREES);

            double turn = headingError * AUTO_AIM_KP + (headingError - previousHeadingError) / loopTime * AUTO_AIM_KD;
            previousHeadingError = headingError;

            //Calculations based on ChatGPT.
            double dx = parkingPose.getX(DistanceUnit.METER) + PoseStorage.parkingCorrectionX - botpose.getX(DistanceUnit.METER);
            double dy = parkingPose.getY(DistanceUnit.METER) + PoseStorage.parkingCorrectionY - botpose.getY(DistanceUnit.METER);

            double heading = botpose.getHeading(AngleUnit.RADIANS); //Radians based on ChatGPT

            double forward =  dx * Math.cos(heading) + dy * Math.sin(heading);
            double strafe  = -dx * Math.sin(heading) + dy * Math.cos(heading);

            follower.setTeleOpDrive(forward, strafe, turn);
            super.update();
        } else {
            //Normal case:
            double forward = -leftStickY * FORWARD_SCALING;
            double strafe = -leftStickX * STRAFE_SCALING;
            double turn = -rightStickX * TURN_SCALING;

            //Overrides if slowMode is enabled.
            if (slowMode) {
                forward = -leftStickY * FORWARD_SCALING_SLOW;
                strafe = -leftStickX * STRAFE_SCALING_SLOW;
                turn = -rightStickX * TURN_SCALING_SLOW;
            }

            if (aimingMode) {
                double headingError = getHeadingError();
                if(robotPose.getX(DistanceUnit.INCH) > Constants.NEAR_PART) {
                    turn = manualAdjustmentFar + headingError * AUTO_AIM_KP + (headingError - previousHeadingError) / loopTime * AUTO_AIM_KD;
                } else {
                    turn = manualAdjustmentNear + headingError * AUTO_AIM_KP + (headingError - previousHeadingError) / loopTime * AUTO_AIM_KD;
                }
                previousHeadingError = headingError;
            }

            follower.setTeleOpDrive(forward, strafe, turn);
            super.update();
        }

    }

    public void adjustAutoAimHeading(double adjustment,Pose2D robotPose) {
        if(robotPose.getX(DistanceUnit.INCH) > Constants.NEAR_PART) {
            manualAdjustmentFar += adjustment;
        } else {
            manualAdjustmentNear += adjustment;
        }
    }
    public void resetAutoAimHeadingAdjustment() {
        manualAdjustmentFar = 0;
        manualAdjustmentNear = 0;
    }

    public goalDistance getGoalDistance() {
        if(getPose().getX(DistanceUnit.INCH) > Constants.NEAR_PART) return goalDistance.FAR;
        else return goalDistance.CLOSE;
    }

    public void setAimingMode(boolean aimingMode) {
        if(this.aimingMode != aimingMode) {
            this.aimingMode = aimingMode;
        }

        //Sets the normal  speed if aiming.
        if(slowMode && this.aimingMode) {
            slowMode = false;
        }
    }

    public boolean getAimingMode() {
        return aimingMode;
    }

    public double getHeadingError() {
        Pose2D botpose = getPose();
        Pose2D target = posetoPose2D(targetPose);

        double dX = target.getX(DistanceUnit.METER) - botpose.getX(DistanceUnit.METER);
        double dY = target.getY(DistanceUnit.METER) - botpose.getY(DistanceUnit.METER);

        double targetHeading = Math.toDegrees(Math.atan2(dY,dX));
        double headingError = targetHeading - botpose.getHeading(AngleUnit.DEGREES);
        if(headingError >= 180.0) {
            headingError -= 360;
        }
        if(headingError <= -180.0) {
            headingError += 360;
        }
        return headingError;
    }

    public void slowDrivetrain() {
        if(!aimingMode) {
            slowMode = true;
        }
    }

    public void normalDrivetrain() {
        slowMode = false;
    }

    public void park() {
        parkingMode = true;
    }

    public void noParking() {
        parkingMode = false;
    }

}
