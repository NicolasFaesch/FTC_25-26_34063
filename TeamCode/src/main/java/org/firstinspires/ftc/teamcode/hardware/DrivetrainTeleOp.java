package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DrivetrainTeleOp extends Drivetrain{
    private static final double FORWARD_SCALING = 1.0;
    private static final double STRAFE_SCALING = 1.0;
    private static final double TURN_SCALING = 1.0;

    // auto-aim PID gains
    private static final double AUTO_AIM_KP = 0.025;
    private static final double AUTO_AIM_KD = 0.0005;

    private boolean aimingMode;

    private double previousHeadingError = 0;

    public DrivetrainTeleOp(HardwareMap hardwareMap, Pose2D startPose) {
        super(hardwareMap, startPose);
        follower.startTeleOpDrive(true);

        aimingMode = false;
    }

    public void update(double leftStickX, double leftStickY, double rightStickX, double loopTime) {
        double forward = -leftStickY*FORWARD_SCALING;
        double strafe = leftStickX*STRAFE_SCALING;
        double turn = rightStickX*TURN_SCALING;

        if(aimingMode) {
            double headingError = getHeadingError();
            turn = headingError * AUTO_AIM_KP + (headingError-previousHeadingError)/loopTime*AUTO_AIM_KD;
        }

        follower.setTeleOpDrive(forward,strafe,turn);
        super.update();
    }

    public void setAimingMode(boolean aimingMode) {
        this.aimingMode = aimingMode;
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
        return headingError;
    }

}
