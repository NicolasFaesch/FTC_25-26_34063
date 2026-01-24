package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "autoTest", group = "Autonomous")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private int pathState;

    private final Pose startPose = new Pose(-160/2.5, 27/2.5, Math.toRadians(90));
    private final Pose pickUp1Pose = new Pose(-30/2.5,10/2.5, Math.toRadians(90));
    private final Pose pickUp2Pose = new Pose(-20/2.5, -39/2.5, Math.toRadians(0));
    private final Pose collecting2Pose = new Pose(45/2.5, -39/2.5, Math.toRadians(0));
    private final Pose shootingPose = new Pose(-80/2.5, 80/2.5, Math.toRadians(135));

    private Path startToPickUp, pickUp, pickUpToScore;

    public void buildPaths(){
        startToPickUp = new Path(new BezierCurve(startPose, pickUp1Pose, pickUp2Pose));
        startToPickUp.setLinearHeadingInterpolation(startPose.getHeading(), pickUp2Pose.getHeading());

        pickUp = new Path(new BezierLine(pickUp2Pose, collecting2Pose));
        pickUp.setLinearHeadingInterpolation(pickUp2Pose.getHeading(), collecting2Pose.getHeading());
        pickUp.setVelocityConstraint(5);

        pickUpToScore = new Path(new BezierLine(collecting2Pose, shootingPose));
        pickUpToScore.setLinearHeadingInterpolation(collecting2Pose.getHeading(), shootingPose.getHeading());
    }
    public void autonomusPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(startToPickUp);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(pickUp);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pickUpToScore);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor intake;
    public DcMotor transfer;
    public DcMotorEx shooterLeft;
    public DcMotorEx shooterRight;

    private enum FeederState {
        IDLE,
        EXTENDING,
        RETRACTING
    }

    FeederState feederState = FeederState.IDLE;
    public Servo feeder_servo;
    double feederRetracted = 0.95; // 0.95
    double feederExtended = 0.6;
    double feederTime = 0.12; // in seconds              0.3
    double feederIdleTime = 0.15; // time the feeder has to wait for ball to come into position  0.7
    double feederStartTime = 0.0;

    public Servo hood;


    @Override
    public  void loop () {
        switch (pathState) {
            case 2:
                intake.setPower(0.7);
                transfer.setPower(-1);
                break;
            case 3:
            case -1:
                intake.setPower(0.7);
                transfer.setPower(-1);
                shooterLeft.setVelocity(3000/60*28);
                shooterRight.setVelocity(3000/60*28);
        }

        switch (feederState) {
            case IDLE:
                if(runtime.seconds() >= feederIdleTime + feederStartTime && pathState == -1) {
                    feeder_servo.setPosition(feederExtended);
                    feederState = FeederState.EXTENDING;
                    feederStartTime = runtime.seconds();
                }
                break;
            case EXTENDING:
                if (runtime.seconds() >= feederTime + feederStartTime) {
                    feeder_servo.setPosition(feederRetracted);
                    feederState = FeederState.RETRACTING;
                    feederStartTime = runtime.seconds();
                }
                break;
            case RETRACTING:
                if (runtime.seconds() >= feederTime + feederStartTime) {
                    feederState = FeederState.IDLE;
                    feederStartTime = runtime.seconds();
                }
                break;
            default:
                break;
        }

        follower.update();
        autonomusPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("yaw", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        feeder_servo = hardwareMap.get(Servo.class, "feeder_servo");
        hood = hardwareMap.get(Servo.class, "hood");

        feeder_servo.setPosition(feederRetracted);
        hood.setPosition(0.65);

        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop(){
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("yaw", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start(){
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){}
}
