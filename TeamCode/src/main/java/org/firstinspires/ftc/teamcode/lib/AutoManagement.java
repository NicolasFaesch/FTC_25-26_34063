package org.firstinspires.ftc.teamcode.lib;

import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

import java.util.*;

public class AutoManagement {

    // ----------------------------------------------------
    // ENUMS
    // ----------------------------------------------------

    public enum Objective {
        SHOOTING_START,
        SPIKE_MARK_CLOSE,
        SPIKE_MARK_MIDDLE,
        SPIKE_MARK_FAR,
        GATE_INTAKING,
        GATE_RELEASING,
        LOADING_ZONE,
        PARK
    }

    public enum Task {
        WAITING,
        SHOOTING,
        INTAKING,
        DRIVE_TO_SHOOT,
        DRIVE_TO_INTAKE,
        DRIVE
    }

    // ----------------------------------------------------
    // TRANSITION KEY
    // ----------------------------------------------------

    private static class Key {
        Objective from;
        Objective to;
        Task task;
        boolean autoClose;

        Key(Objective from, Objective to, Task task, boolean autoClose) {
            this.from = from;
            this.to = to;
            this.task = task;
            this.autoClose = autoClose;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Key)) return false;
            Key k = (Key) o;
            return from == k.from &&
                    to == k.to &&
                    task == k.task &&
                    autoClose == k.autoClose;
        }

        @Override
        public int hashCode() {
            return Objects.hash(from, to, task, autoClose);
        }
    }

    // ----------------------------------------------------
    // FIELDS
    // ----------------------------------------------------

    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private final List<Objective> objectives = new ArrayList<>();
    private final List<Task> taskList = new ArrayList<>();
    private final Map<Key, Path> pathMap = new HashMap<>();

    private int objectiveIndex = 0;
    private int taskIndex = 0;

    private Objective previousObjective;
    private Objective currentObjective;
    private Objective nextObjective;

    private Task currentTask;

    // ----------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------


    private RobotAuto robotAuto;
    private boolean autoClose;

    public AutoManagement(RobotAuto robotAuto, boolean autoClose) {

        this.robotAuto = robotAuto;
        this.autoClose = autoClose;

        AutoPaths autoPaths = new AutoPaths();
        autoPaths.buildPaths();

        buildPathMap();

        this.autoTimer = new Timer();
    }


    // ----------------------------------------------------
    // OBJECTIVES
    // ----------------------------------------------------

    public void addObjective(Objective o) {
        objectives.add(o);
    }


    public void start() {

        if (objectives.isEmpty()) {
            throw new IllegalStateException("No objectives defined");
        }

        objectiveIndex = 0;
        currentObjective = objectives.get(objectiveIndex);

        previousObjective = currentObjective;

        nextObjective = (objectives.size() > 1)
                ? objectives.get(objectiveIndex + 1)
                : null;

        buildTaskList();

        if (taskList.isEmpty()) {
            throw new IllegalStateException("No tasks generated for " + currentObjective);
        }

        taskIndex = 0;
        currentTask = taskList.get(taskIndex);

        autoTimer.resetTimer();
        executeCurrentTask();
    }


    private void nextObjective() {
        previousObjective = currentObjective;
        objectiveIndex++;

        if (objectiveIndex >= objectives.size())
            return;

        currentObjective = objectives.get(objectiveIndex);
        nextObjective = (objectiveIndex + 1 < objectives.size())
                ? objectives.get(objectiveIndex + 1) : null;

        buildTaskList();
        taskIndex = 0;
        currentTask = taskList.get(0);
        executeCurrentTask();
    }

    // ----------------------------------------------------
    // TASK BUILDING
    // ----------------------------------------------------

    private void buildTaskList() {

        taskList.clear();

        switch (currentObjective) {

            case SPIKE_MARK_CLOSE:
            case SPIKE_MARK_MIDDLE:
            case SPIKE_MARK_FAR:
            case LOADING_ZONE:
            case GATE_INTAKING:
                taskList.add(Task.DRIVE_TO_INTAKE);
                taskList.add(Task.INTAKING);

                if (nextObjective == Objective.GATE_RELEASING ||
                        nextObjective == Objective.PARK) {

                    taskList.add(Task.DRIVE_TO_SHOOT);
                    taskList.add(Task.SHOOTING);
                }
                break;

            case GATE_RELEASING:
                taskList.add(Task.DRIVE);
                taskList.add(Task.WAITING);
                break;

            case SHOOTING_START:
                if (!autoClose)
                    taskList.add(Task.DRIVE_TO_SHOOT);

                taskList.add(Task.SHOOTING);
                break;

            case PARK:
                taskList.add(Task.DRIVE);
                if (autoClose)
                    taskList.add(Task.SHOOTING);
                taskList.add(Task.WAITING);
                break;
        }
    }

    // ----------------------------------------------------
    // TASK FLOW
    // ----------------------------------------------------

    public Timer autoTimer;

    public void nextTask() {
        autoTimer.resetTimer();

        if (taskIndex >= taskList.size() - 1) {
            nextObjective();
            return;
        }

        taskIndex++;
        currentTask = taskList.get(taskIndex);
        executeCurrentTask();
    }

    private boolean shootStarted;

    private void executeCurrentTask() {
        Key key = new Key(previousObjective, currentObjective, currentTask, autoClose);
        Path path = pathMap.get(key);

        if (path == null) {
            key = new Key(null, currentObjective, currentTask, autoClose);
            path = pathMap.get(key);
        }
        if (!robotAuto.drivetrainAuto.pathStarted && autoTimer.getElapsedTimeSeconds() > 0.15) {
            if (path != null){ robotAuto.drivetrainAuto.followPath(path);}
        }
    }

    // ----------------------------------------------------
    // UPDATE LOOP
    // ----------------------------------------------------

    public void update() {

        robotAuto.drivetrainAuto.update();

        if (currentTask == null)
            return;

        switch (currentTask) {

            case DRIVE:
            case DRIVE_TO_INTAKE:
            case DRIVE_TO_SHOOT:
                if (robotAuto.drivetrainAuto.pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.setState(Robot.State.STORING);
                    nextTask();
                }
                break;

            case INTAKING:
                if (robotAuto.drivetrainAuto.pathStarted && !robotAuto.drivetrainAuto.isFollowerBusy()) {
                    robotAuto.setState(Robot.State.INTAKING);
                    nextTask();
                }
                break;

            case SHOOTING:
                if (!shootStarted) {
                    robotAuto.resetShots();
                    robotAuto.setState(Robot.State.SHOOTING);
                    shootStarted = true;
                }
                if (robotAuto.getBallsShot() >= 3) {
                    nextTask();
                }
                break;
            case WAITING:
                robotAuto.setState(Robot.State.IDLE);
                nextTask();
                break;
        }
    }

    // ----------------------------------------------------
    // PATH MAP
    // ----------------------------------------------------

    private void buildPathMap() {

        // =====================================================
        // DRIVE_TO_INTAKE
        // =====================================================

        // ----- SPIKE_MARK_CLOSE -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToIntakingClose);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToIntakingClose);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.shootCloseToIntakingClose);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.shootFarToIntakingClose);


        // ----- SPIKE_MARK_MIDDLE -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToIntakingMiddle);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToIntakingMiddle);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.shootCloseToIntakingMiddle);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.shootFarToIntakingMiddle);


        // ----- SPIKE_MARK_FAR -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToIntakingFar);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToIntakingFar);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.shootCloseToIntakingFar);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.shootFarToIntakingFar);


        // ----- GATE_INTAKING -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.GATE_INTAKING, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToGateIntaking);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.GATE_INTAKING, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToGateIntaking);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.GATE_INTAKING, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.shootCloseToGateIntaking);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.GATE_INTAKING, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.shootFarToGateIntaking);


        // ----- GATE_RELEASING -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToGateReleasing);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToGateReleasing);


        // ----- LOADING_ZONE -----
        pathMap.put(new Key(Objective.SHOOTING_START, Objective.LOADING_ZONE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.startCloseToLoadingZone);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.LOADING_ZONE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.startFarToLoadingZone);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.LOADING_ZONE, Task.DRIVE_TO_INTAKE, true),
                AutoPaths.shootCloseToLoadingZone);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.LOADING_ZONE, Task.DRIVE_TO_INTAKE, false),
                AutoPaths.shootFarToLoadingZone);



        // =====================================================
        // INTAKING
        // =====================================================

        pathMap.put(new Key(null, Objective.SPIKE_MARK_CLOSE, Task.INTAKING, true),
                AutoPaths.intakingClose);

        pathMap.put(new Key(null, Objective.SPIKE_MARK_MIDDLE, Task.INTAKING, true),
                AutoPaths.intakingMiddle);

        pathMap.put(new Key(null, Objective.SPIKE_MARK_FAR, Task.INTAKING, true),
                AutoPaths.intakingFar);

        pathMap.put(new Key(null, Objective.LOADING_ZONE, Task.INTAKING, true),
                AutoPaths.intakingLoadingZone);



        // =====================================================
        // DRIVE_TO_SHOOT
        // =====================================================

        // ----- CLOSE -----
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.intakingCloseToShootClose);

        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.intakingMiddleToShootClose);

        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.intakingFarToShootClose);

        pathMap.put(new Key(Objective.GATE_INTAKING, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.gateIntakingToShootClose);

        pathMap.put(new Key(Objective.LOADING_ZONE, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.loadingZoneToShootClose);

        pathMap.put(new Key(Objective.GATE_RELEASING, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.gateReleasingToShootClose);

        pathMap.put(new Key(Objective.SHOOTING_START, null, Task.DRIVE_TO_SHOOT, true),
                AutoPaths.startCloseToShootClose);


        // ----- FAR -----
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.intakingCloseToShootFar);

        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.intakingMiddleToShootFar);

        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.intakingFarToShootFar);

        pathMap.put(new Key(Objective.GATE_INTAKING, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.gateIntakingToShootFar);

        pathMap.put(new Key(Objective.LOADING_ZONE, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.loadingZoneToShootFar);

        pathMap.put(new Key(Objective.GATE_RELEASING, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.gateReleasingToShootFar);

        pathMap.put(new Key(Objective.SHOOTING_START, null, Task.DRIVE_TO_SHOOT, false),
                AutoPaths.startFarToShootFar);



        // =====================================================
        // DRIVE (PARK + SPECIAL)
        // =====================================================

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.PARK, Task.DRIVE, false),
                AutoPaths.gateReleasingToParkedFar);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.PARK, Task.DRIVE, false),
                AutoPaths.startFarToParkedFar);

        pathMap.put(new Key(null, Objective.PARK, Task.DRIVE, false),
                AutoPaths.shootFarToParkedFar);

        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Objective.PARK, Task.DRIVE, true),
                AutoPaths.intakingCloseToShootCloseParked);

        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Objective.PARK, Task.DRIVE, true),
                AutoPaths.intakingMiddleToShootCloseParked);

        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Objective.PARK, Task.DRIVE, true),
                AutoPaths.intakingFarToShootCloseParked);

        pathMap.put(new Key(Objective.GATE_INTAKING, Objective.PARK, Task.DRIVE, true),
                AutoPaths.gateIntakingToShootCloseParked);

        pathMap.put(new Key(Objective.LOADING_ZONE, Objective.PARK, Task.DRIVE, true),
                AutoPaths.loadingZoneToShootCloseParked);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.PARK, Task.DRIVE, true),
                AutoPaths.gateReleasingToShootCloseParked);

        pathMap.put(new Key(Objective.SHOOTING_START, Objective.PARK, Task.DRIVE, true),
                AutoPaths.startCloseToShootCloseParked);



    // gute Überarbeiten

        // SHOOT CLOSE → GATE_RELEASING
        pathMap.put(new Key(null, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.shootCloseToGateReleasing);

        // SHOOT FAR → GATE_RELEASING
        pathMap.put(new Key(null, Objective.GATE_RELEASING, Task.DRIVE, false),
                AutoPaths.shootFarToGateReleasing);


        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingCloseToGateReleasing);

        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingMiddleToGateReleasing);

        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingFarToGateReleasing);

        pathMap.put(new Key(Objective.LOADING_ZONE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.loadingZoneToGateReleasing);


        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_CLOSE, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingClose);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingMiddle);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_FAR, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingFar);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.LOADING_ZONE, Task.DRIVE, false),
                AutoPaths.gateReleasingToLoadingZone);


        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingCloseToGateReleasing);

        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingMiddleToGateReleasing);

        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.intakingFarToGateReleasing);

        pathMap.put(new Key(Objective.LOADING_ZONE, Objective.GATE_RELEASING, Task.DRIVE, true),
                AutoPaths.loadingZoneToGateReleasing);


        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_CLOSE, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingClose);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingMiddle);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.SPIKE_MARK_FAR, Task.DRIVE, false),
                AutoPaths.gateReleasingToIntakingFar);

        pathMap.put(new Key(Objective.GATE_RELEASING, Objective.LOADING_ZONE, Task.DRIVE, false),
                AutoPaths.gateReleasingToLoadingZone);
    }

    public void updateTelemetry() {
        Pose2D botPose = robotAuto.drivetrainAuto.getPose();
        String position = String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.1f",
                botPose.getX(DistanceUnit.INCH),
                botPose.getY(DistanceUnit.INCH),
                Math.toDegrees(botPose.getHeading(AngleUnit.DEGREES)));

        panelsTelemetry.addLine("=== Task ===");
        panelsTelemetry.addData("previousObjective", previousObjective);
        panelsTelemetry.addData("currentTask", currentTask);

        panelsTelemetry.addLine("=== ROBOT STATES ===");
        panelsTelemetry.addData("Robot State", robotAuto.getState());
        panelsTelemetry.addData("Intake State", robotAuto.intake.getState());
        panelsTelemetry.addData("Transfer State", robotAuto.transfer.getState());
        panelsTelemetry.addData("Shooter State", robotAuto.shooter.getState());

        panelsTelemetry.addLine("=== POSE ===");
        if(!robotAuto.isUsingLimelight()) {
            panelsTelemetry.addData("Odometry", position);
        } else {
            panelsTelemetry.addData("Limelight", position);
        }
        panelsTelemetry.addData("Distance", robotAuto.drivetrainAuto.getDistance());

        panelsTelemetry.addLine("=== SHOOTER ===");
        if(robotAuto.shooter.getManualOverride()) {
            panelsTelemetry.addLine("Shooter: MANUAL OVERRIDE");
            panelsTelemetry.addData("Hood Position (manual)", robotAuto.shooter.getHoodPositionManual());
            panelsTelemetry.addData("Target Velocity (manual)", robotAuto.shooter.getShooterTargetVelocityManual());
        } else {
            panelsTelemetry.addData("Hood Position", robotAuto.shooter.getHoodPosition());
            panelsTelemetry.addData("Target Velocity", robotAuto.shooter.getShooterTargetVelocity());
        }
        panelsTelemetry.addData("Current Velocity", robotAuto.shooter.getShooterVelocity());

        panelsTelemetry.update();
    }
}