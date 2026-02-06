package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

import java.util.Locale;

@Autonomous(name = "Auto RED Close", group = "Red")
public class AutoRedClose extends OpMode {
    RobotAuto robotAuto;
    ElapsedTime runtime = new ElapsedTime();
    private Timer autoTimer, startTimer;

    static final int START_WAITING_TIMER = 0;
    double previousTime = 0;

    final Pose startPose = new Pose(-168/2.54, 30/2.54, Math.toRadians(90));
    final Pose shoot0Pose = new Pose(-94/2.54, 62/2.54, Math.toRadians(120));
    final Pose pickUp1PoseBefore = new Pose(-35/2.54,3/2.54, Math.toRadians(90));
    final Pose pickUp1PoseAfter = new Pose(-34/2.54, 100/2.54, Math.toRadians(90));
    final Pose shoot1Pose = new Pose(-112/2.54, 73/2.54, Math.toRadians(120));

    enum AutoState {
        WAITING,
        START_TO_SHOOT0,
        SHOOT0,
        SHOOT0_TO_PICKUP1,
        PICKUP1,
        PICKUP1_TO_SHOOT1,
        SHOOT1,
        DONE
    }

    AutoState autoState;

    Path startToShoot0, shoot0ToPickup1, pickup1, pickup1ToShoot1;

    void buildPaths(){
        startToShoot0 = new Path(new BezierLine(startPose, shoot0Pose));
        startToShoot0.setLinearHeadingInterpolation(startPose.getHeading(), shoot0Pose.getHeading());

        shoot0ToPickup1 = new Path(new BezierLine(shoot0Pose, pickUp1PoseBefore));
        shoot0ToPickup1.setLinearHeadingInterpolation(shoot0Pose.getHeading(), pickUp1PoseBefore.getHeading());

        pickup1 = new Path(new BezierLine(pickUp1PoseBefore, pickUp1PoseAfter));
        pickup1.setLinearHeadingInterpolation(pickUp1PoseBefore.getHeading(), pickUp1PoseAfter.getHeading());
        pickup1.setVelocityConstraint(2);

        pickup1ToShoot1 = new Path(new BezierLine(pickUp1PoseAfter, shoot1Pose));
        pickup1ToShoot1.setLinearHeadingInterpolation(pickUp1PoseAfter.getHeading(),shoot1Pose.getHeading());
    }

    void autonomousPathUpdate(){
        switch (autoState) {
            case WAITING:
                if (autoTimer.getElapsedTime() >= START_WAITING_TIMER) {
                    robotAuto.drivetrainAuto.followPathAndHold(startToShoot0);
                    setAutoState(AutoState.START_TO_SHOOT0);
                }
                break;
            case START_TO_SHOOT0:
                if(robotAuto.drivetrainAuto.isAtEnd()) {
                    robotAuto.resetShots();
                    robotAuto.setState(Robot.State.SHOOTING);
                    setAutoState(AutoState.SHOOT0);
                }
                break;
            case SHOOT0:
                if(robotAuto.getBallsShot() >= 3) {
                    robotAuto.setState(Robot.State.IDLE);
                    robotAuto.drivetrainAuto.followPath(shoot0ToPickup1);
                    setAutoState(AutoState.SHOOT0_TO_PICKUP1);
                }
                break;
            case SHOOT0_TO_PICKUP1:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.setState(Robot.State.INTAKING);
                    robotAuto.drivetrainAuto.followPath(pickup1);
                    setAutoState(AutoState.PICKUP1);
                }
                break;
            case PICKUP1:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.setState(Robot.State.STORING);
                    robotAuto.drivetrainAuto.followPathAndHold(pickup1ToShoot1);
                    setAutoState(AutoState.PICKUP1_TO_SHOOT1);
                }
                break;
            case PICKUP1_TO_SHOOT1:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.resetShots();
                    robotAuto.setState(Robot.State.SHOOTING);
                    setAutoState(AutoState.SHOOT1);
                }
                break;
            case SHOOT1:
                if(robotAuto.getBallsShot() >= 3){
                    robotAuto.setState(Robot.State.IDLE);
                    setAutoState(AutoState.DONE);
                }
                break;
            case DONE:
                break;
        }
    }

    void setAutoState(AutoState autoState) {
        this.autoState = autoState;
        autoTimer.resetTimer();
    }



    @Override
    public void init() {
        robotAuto = new RobotAuto(hardwareMap, Robot.Alliance.RED, new Pose2D(DistanceUnit.INCH, startPose.getX(), startPose.getY(), AngleUnit.RADIANS, startPose.getHeading()));
        autoTimer = new Timer();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        buildPaths();
        setAutoState(AutoState.WAITING);
    }

    @Override
    public void loop() {
        // clear cache for bulk reading
        for (LynxModule module : this.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        double newTime = getRuntime();
        double loopTime = newTime-previousTime;
        previousTime = newTime;

        try {
            robotAuto.update();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        autonomousPathUpdate();

        Pose2D botpose = robotAuto.drivetrainAuto.getPose();
        String position = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
        String velocity = String.format(Locale.US,"XVel: %.3f, YVel: %.3f, AVel: %.3f", robotAuto.drivetrainAuto.getVelocityX(), robotAuto.drivetrainAuto.getVelocityY(), robotAuto.drivetrainAuto.getAngularVelocity());

        telemetry.addData("Loop Time [ms]", loopTime*1000);

        telemetry.addLine("");

        telemetry.addLine("STATES:");
        telemetry.addData("Robot State", robotAuto.getState());
        telemetry.addData("Intake State", robotAuto.intake.getState());
        telemetry.addData("Transfer State", robotAuto.transfer.getState());
        telemetry.addData("Shooter State", robotAuto.shooter.getState());

        telemetry.addLine("");

        // Robot pose telemetry
        if(!robotAuto.isUsingLimelight()) {
            telemetry.addData("Pose (Odometry)", position);
        } else {
            telemetry.addData("Pose (Limelight)", position);
        }
        telemetry.addData("Distance", robotAuto.drivetrainAuto.getDistance());


        telemetry.addLine("");

        // Shooter telemetry
        if(robotAuto.shooter.getManualOverride()) {
            telemetry.addLine("Shooter: MANUAL OVERRIDE");
            telemetry.addData("Hood Position (manual)", robotAuto.shooter.getHoodPositionManual());
            telemetry.addData("Shooter Target Velocity (manual)", robotAuto.shooter.getShooterTargetVelocityManual());
        } else {
            telemetry.addLine("Shooter:");
            telemetry.addData("Hood Position", robotAuto.shooter.getHoodPosition());
            telemetry.addData("Shooter Target Velocity", robotAuto.shooter.getShooterTargetVelocity());
        }
        telemetry.addData("Shooter Velocity", robotAuto.shooter.getShooterVelocity());


        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
