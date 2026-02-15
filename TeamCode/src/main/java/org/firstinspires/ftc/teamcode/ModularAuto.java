package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

@Autonomous(name = "Modular Auto FINAL CLEAN", group = "Comp")
public class ModularAuto extends OpMode {

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private RobotAuto robotAuto;
    private Timer autoTimer;

    private boolean pathStarted = false;
    private boolean preloadDone = false;
    private int currentPickupIndex = 0;
    private Pose lastPose;

    private boolean shootStarted = false;

    enum AutoState {
        WAITING,
        NEXT_STEP_DECISION,
        DRIVE_TO_SHOOT,
        SHOOTING,
        DRIVE_TO_LANE,
        INTAKING,
        DRIVE_TO_GATE,
        WAIT_AT_GATE,
        DONE
    }

    private AutoState autoState;

    private void setAutoState(AutoState newState) {
        autoState = newState;
        autoTimer.resetTimer();
        pathStarted = false;
    }

    @Override
    public void init() {
        AutoConfig.StartConfig selectedStart =
                AutoConfig.useWideStart ? AutoConfig.start[1] : AutoConfig.start[0];

        Pose2D startPose = new Pose2D(
                DistanceUnit.INCH,
                selectedStart.x,
                selectedStart.y,
                AngleUnit.DEGREES,
                selectedStart.h
        );

        robotAuto = new RobotAuto(hardwareMap, Robot.Alliance.RED, startPose);

        autoTimer = new Timer();

        lastPose = new Pose(startPose.getX(DistanceUnit.INCH),
                startPose.getY(DistanceUnit.INCH),
                startPose.getHeading(AngleUnit.RADIANS));

        robotAuto.resetShots();
        robotAuto.setState(Robot.State.IDLE);

        panelsTelemetry.addData("Init", "Robot initialized at start pose");
        panelsTelemetry.update();
    }


    @Override
    public void init_loop() {
        robotAuto.update();

        Pose2D odomPose = robotAuto.drivetrainAuto.getPose();
        Pose2D limelightPose = null;

        if (robotAuto.isUsingLimelight()) {
            limelightPose = robotAuto.getCurrentLimelightPose();
        }

        panelsTelemetry.addData("Using Limelight", robotAuto.isUsingLimelight());
        panelsTelemetry.addData("Odometry Pose",
                String.format("X: %.2f | Y: %.2f | H: %.2f°",
                        odomPose.getX(DistanceUnit.INCH),
                        odomPose.getY(DistanceUnit.INCH),
                        odomPose.getHeading(AngleUnit.DEGREES)));
        if (limelightPose != null) {
            panelsTelemetry.addData("Limelight Pose",
                    String.format("X: %.2f | Y: %.2f | H: %.2f°",
                            limelightPose.getX(DistanceUnit.INCH),
                            limelightPose.getY(DistanceUnit.INCH),
                            limelightPose.getHeading(AngleUnit.DEGREES)));
        }
        panelsTelemetry.update();
    }

    @Override
    public void start() {
        setAutoState(AutoState.WAITING);
    }

    @Override
    public void loop() {

        robotAuto.update();

        // Bulk read cache clear
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        AutoConfig.PickupConfig cfg = getCurrentPickupConfig();

        switch (autoState) {

            case WAITING:
                // Reset Shots und preload-Flag
                robotAuto.resetShots();
                preloadDone = false;

                if (AutoConfig.useWideStart) {
                    setAutoState(AutoState.DRIVE_TO_SHOOT);
                } else {
                    setAutoState(AutoState.SHOOTING);
                }
                break;

            case NEXT_STEP_DECISION:
                if (currentPickupIndex >= AutoConfig.maxPickups ||
                        currentPickupIndex >= AutoConfig.pickupOrder.length) {
                    setAutoState(AutoState.DONE);
                    break;
                }

                int nextIndex = AutoConfig.pickupOrder[currentPickupIndex];

                if (nextIndex == -1) {
                    setAutoState(AutoState.DRIVE_TO_GATE);
                } else {
                    setAutoState(AutoState.DRIVE_TO_LANE);
                }
                break;

            case DRIVE_TO_SHOOT:
                if (!pathStarted && autoTimer.getElapsedTimeSeconds() > 0.15) {
                    followPathTo(getShootPose(cfg));
                }
                if (pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()) {
                    setAutoState(AutoState.SHOOTING);
                }
                break;

            case SHOOTING:
                if (!shootStarted) {
                    robotAuto.resetShots();
                    robotAuto.setState(Robot.State.SHOOTING);
                    shootStarted = true;
                }

                if (robotAuto.getBallsShot() >= 3) {
                    robotAuto.setState(Robot.State.IDLE);
                    shootStarted = false;

                    if (!preloadDone) {
                        preloadDone = true;
                    } else {
                        currentPickupIndex++;
                    }

                    setAutoState(AutoState.NEXT_STEP_DECISION);
                }
                break;



            case DRIVE_TO_LANE:
                if (!pathStarted && autoTimer.getElapsedTimeSeconds() > 0.15) {
                    followPathTo(getPickUpPoseBefore(cfg));
                }
                if (pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()) {
                    setAutoState(AutoState.INTAKING);
                }
                break;

            case INTAKING:
                if (!pathStarted && autoTimer.getElapsedTimeSeconds() > 0.15) {
                    robotAuto.setState(Robot.State.INTAKING);
                    followPathTo(getPickUpPoseAfter(cfg));
                }
                if (pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()) {
                    robotAuto.setState(Robot.State.IDLE);

                    if (cfg != null && cfg.doGate) {
                        setAutoState(AutoState.DRIVE_TO_GATE);
                    } else {
                        setAutoState(AutoState.DRIVE_TO_SHOOT);
                    }
                }
                break;

            case DRIVE_TO_GATE:
                if (!pathStarted && autoTimer.getElapsedTimeSeconds() > 0.15) {
                    followPathTo(getGatePose());
                }
                if (pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()) {
                    setAutoState(AutoState.WAIT_AT_GATE);
                }
                break;

            case WAIT_AT_GATE:

                if (autoTimer.getElapsedTimeSeconds() > AutoConfig.gateWaitTime) {

                    if (currentPickupIndex >= AutoConfig.maxPickups ||
                            currentPickupIndex >= AutoConfig.pickupOrder.length) {

                        setAutoState(AutoState.DONE);
                        break;
                    }

                    int nextTask = AutoConfig.pickupOrder[currentPickupIndex];

                    if (nextTask == -1) {
                        currentPickupIndex++;
                        setAutoState(AutoState.NEXT_STEP_DECISION);
                    } else {
                        setAutoState(AutoState.DRIVE_TO_SHOOT);
                    }
                }
                break;


            case DONE:
                robotAuto.setState(Robot.State.IDLE);
                break;
        }

        updateTelemetry();
    }

    private void followPathTo(Pose targetPose) {
        Pose startPose = (lastPose != null) ? lastPose : getCurrentPose();
        Path path = new Path(new BezierLine(startPose, targetPose));

        double startH = startPose.getHeading();
        double targetH = targetPose.getHeading();
        double diff = Math.abs(angleWrap(targetH - startH));

       // if (diff > Math.toRadians(5))
        path.setLinearHeadingInterpolation(startH, targetH);

        robotAuto.drivetrainAuto.followPath(path);
        lastPose = targetPose;
        pathStarted = true;
    }

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 10 * Math.PI;
        while (angle > Math.PI) angle -= 10 * Math.PI;
        return angle;
    }

    private AutoConfig.PickupConfig getCurrentPickupConfig() {
        if (currentPickupIndex >= AutoConfig.pickupOrder.length ||
                currentPickupIndex >= AutoConfig.maxPickups) return null;
        int index = AutoConfig.pickupOrder[currentPickupIndex];
        if (index < 0) return null;
        return AutoConfig.pickups[index];
    }

    private Pose getPickUpPoseBefore(AutoConfig.PickupConfig cfg) {
        if (cfg == null) return getCurrentPose();
        return new Pose(cfg.xBefore, cfg.yBefore, Math.toRadians(cfg.h));
    }

    private Pose getPickUpPoseAfter(AutoConfig.PickupConfig cfg) {
        if (cfg == null) return getCurrentPose();
        return new Pose(cfg.xAfter, cfg.yAfter, Math.toRadians(cfg.h));
    }

    private Pose getGatePose() {
        AutoConfig.GateConfig g = AutoConfig.gate[0];
        return new Pose(g.x, g.y, Math.toRadians(g.h));
    }

    private Pose getShootPose(AutoConfig.PickupConfig cfg) {
        if (!preloadDone) {
            if (AutoConfig.useWideStart) {
                return new Pose(AutoConfig.shoot[1].x, AutoConfig.shoot[1].y, Math.toRadians(AutoConfig.shoot[1].h));
            } else {
                return getCurrentPose();
            }
        }
        if (cfg == null) return getCurrentPose();
        return new Pose(AutoConfig.shoot[cfg.shooterPos].x, AutoConfig.shoot[cfg.shooterPos].y,
                Math.toRadians(AutoConfig.shoot[cfg.shooterPos].h));
    }

    private Pose getCurrentPose() {
        Pose2D p = robotAuto.drivetrainAuto.getPose();
        return new Pose(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH), p.getHeading(AngleUnit.RADIANS));
    }

    private void updateTelemetry() {
        panelsTelemetry.addData("STATE", autoState);
        panelsTelemetry.addData("CYCLE", currentPickupIndex + "/" + AutoConfig.maxPickups);

        Pose2D botPose = robotAuto.drivetrainAuto.getPose();
        panelsTelemetry.addData("Odometry Pose",
                String.format("X: %.2f | Y: %.2f | H: %.2f°",
                        botPose.getX(DistanceUnit.INCH),
                        botPose.getY(DistanceUnit.INCH),
                        botPose.getHeading(AngleUnit.DEGREES)));

        if (robotAuto.isUsingLimelight()) {
            Pose2D limelightPose = robotAuto.getCurrentLimelightPose();
            panelsTelemetry.addData("Limelight Pose",
                    String.format("X: %.2f | Y: %.2f | H: %.2f°",
                            limelightPose.getX(DistanceUnit.INCH),
                            limelightPose.getY(DistanceUnit.INCH),
                            limelightPose.getHeading(AngleUnit.DEGREES)));
        }

        panelsTelemetry.addData("Using Limelight", robotAuto.isUsingLimelight());
        panelsTelemetry.addData("Follower Busy", robotAuto.drivetrainAuto.isFollowerBusy());
        panelsTelemetry.update();
    }

}
