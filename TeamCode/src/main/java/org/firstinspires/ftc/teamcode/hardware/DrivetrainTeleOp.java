package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

@Configurable
public class DrivetrainTeleOp extends Drivetrain{
    public static double FORWARD_SCALING = 1.0;
    public static double STRAFE_SCALING = 1.0;
    public static double TURN_SCALING = 0.7;
    public static double PARKING_SCALING = 0.3;

    public static double PARKING_CORRECTION = 0.5; // in inches

    private boolean parkingMode;
    private boolean autoDrive;

    private Pose parkingPose;
    private Pose parkingControlPoint;
    private Pose gatePose;
    private Pose gateControlPoint;


    public DrivetrainTeleOp(HardwareMap hardwareMap) {
        super(hardwareMap, PoseStorage.currentPose);
        follower.startTeleOpDrive(false);
        parkingMode = false;
        autoDrive = false;
    }


    public void update(double leftStickX, double leftStickY, double rightStickX) {
        double forward = -leftStickY * FORWARD_SCALING;
        double strafe = -leftStickX * STRAFE_SCALING;
        double turn = -rightStickX * TURN_SCALING;

        if(autoDrive && (Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)) > 0.2) {
            setAutoDrive(false);
        }

        if(parkingMode) {
            forward *= PARKING_SCALING;
            strafe *= PARKING_SCALING;
            turn *= PARKING_SCALING;
        }

        if(!autoDrive) {
            follower.setTeleOpDrive(forward, strafe, turn);
        }

        super.update();
    }

    public void setParkingMode(boolean parkingMode) {
        this.parkingMode = parkingMode;
    }

    public boolean getParkingMode() {
        return parkingMode;
    }

    public void setAutoDrive(boolean autoDrive) {
        this.autoDrive = autoDrive;
        if (!this.autoDrive) {
            follower.startTeleOpDrive(false);
        } else {
            //follower.activateDrive();
        }
    }

    public boolean  getAutoDrive() {
        return autoDrive;
    }

    public void setParkingPose(Pose parkingPose, Pose parkingControlPoint) {
        this.parkingPose = parkingPose;
        this.parkingControlPoint = parkingControlPoint;
    }

    public void setGatePose(Pose gatePose, Pose gateControlPoint) {
        this.gatePose = gatePose;
        this.gateControlPoint = gateControlPoint;
    }

    public Pose getCurrentPose() {
        return new Pose(PoseStorage.currentPose.getX(DistanceUnit.INCH), PoseStorage.currentPose.getY(DistanceUnit.INCH), PoseStorage.currentPose.getHeading(AngleUnit.RADIANS));
    }

    public void autoDriveToGate() {
        setAutoDrive(true);
        Path path = new Path(new BezierCurve(getCurrentPose(), gateControlPoint, gatePose));
        path.setBrakingStrength(AutoConstants.GATE_BRAKE_STRENGTH);
        path.setLinearHeadingInterpolation(getCurrentPose().getHeading(), gatePose.getHeading());
        follower.followPath(path, true);
    }

    public void changeParkingPoseX(boolean increaseX) {
        parkingPose = new Pose(parkingPose.getX() + (increaseX ? 1 : -1) * PARKING_CORRECTION, parkingPose.getY(), parkingPose.getHeading());
        if(autoDrive)
            updateParkingTrack();
    }

    public void changeParkingPoseY(boolean increaseY) {
        parkingPose = new Pose(parkingPose.getX(), parkingPose.getY() + (increaseY ? 1 : -1) * PARKING_CORRECTION, parkingPose.getHeading());
        if(autoDrive)
            updateParkingTrack();
    }

    public void autoDriveToPark() {
        setAutoDrive(true);
        setParkingMode(true);
        Path path = new Path(new BezierCurve(getCurrentPose(), parkingControlPoint, parkingPose));
        path.setConstantHeadingInterpolation(parkingPose.getHeading());
        follower.followPath(path, true);
    }

    public void updateParkingTrack() {
        Path path = new Path(new BezierLine(getCurrentPose(), parkingPose));
        path.setConstantHeadingInterpolation(parkingPose.getHeading());
        follower.followPath(path, true);
    }


}
