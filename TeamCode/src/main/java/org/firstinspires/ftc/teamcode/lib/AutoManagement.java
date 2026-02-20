package org.firstinspires.ftc.teamcode.lib;


import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;

import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;
import com.bylazar.field.Style;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

import java.util.*;
import java.util.concurrent.TimeUnit;

public class AutoManagement {

    // Feld zum Zeichnen
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final FieldManager panelsField;



    // ----------------------------------------------------
    // ENUMS
    // ----------------------------------------------------

    public enum Objective {
        SHOOT_START,
        SPIKE_MARK_CLOSE,
        SPIKE_MARK_MIDDLE,
        SPIKE_MARK_FAR,
        GATE_INTAKING,
        GATE_RELEASING,
        LOADING_ZONE,
        SHOOT,
        SHOOT_END,
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
        Objective previousObjective;
        Task currentTask;
        Objective currentObjective;
        boolean autoClose;

        Key(Objective previousObjective, Task currentTask, Objective currentObjective, boolean autoClose) {
            if(currentTask == Task.INTAKING) {
                this.previousObjective = null;
            } else {
                this.previousObjective = previousObjective;
            }
            this.currentObjective = currentObjective;
            this.currentTask = currentTask;
            this.autoClose = autoClose;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Key)) return false;
            Key k = (Key) o;
            return previousObjective == k.previousObjective &&
                    currentObjective == k.currentObjective &&
                    currentTask == k.currentTask &&
                    autoClose == k.autoClose;
        }

        @Override
        public int hashCode() {
            return Objects.hash(previousObjective, currentObjective, currentTask, autoClose);
        }
    }


    // ----------------------------------------------------
    // FIELDS
    // ----------------------------------------------------

    private final List<Objective> objectives = new ArrayList<>();
    private final List<Task> taskList = new ArrayList<>();
    private final Map<Key, Path> pathMap = new HashMap<>();

    private int objectiveIndex = 0;
    private int taskIndex = 0;

    private Objective previousObjective;
    private Objective currentObjective;

    private Task currentTask;

    private Timing.Timer gateReleasingTimer = new Timing.Timer(AutoConstants.GATE_RELEASING_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer gateIntakingTimer = new Timing.Timer(AutoConstants.GATE_INTAKING_TIME, TimeUnit.MILLISECONDS);

    // ----------------------------------------------------
    // CONSTRUCTOR
    // ----------------------------------------------------


    private RobotAuto robotAuto;
    private boolean autoClose;

    public AutoManagement(RobotAuto robotAuto, boolean autoClose) {

        this.robotAuto = robotAuto;
        this.autoClose = autoClose;

        this.panelsField = PanelsField.INSTANCE.getField();
        this.panelsField.init();

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

        previousObjective = null;

        buildTaskList();

        if (taskList.isEmpty()) {
            throw new IllegalStateException("No tasks generated for " + currentObjective);
        }

        taskIndex = -1; // for start
        nextTask();
    }


    private void nextObjective() {
        // Hier bleibt das previousObjective erhalten, bis das neue gesetzt wird
        previousObjective = currentObjective;
        objectiveIndex++;

        if (objectiveIndex >= objectives.size()) return;

        currentObjective = objectives.get(objectiveIndex);

        buildTaskList();
        taskIndex = -1;

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
                taskList.add(Task.DRIVE_TO_INTAKE);
                taskList.add(Task.INTAKING);
                break;
            case GATE_RELEASING:
            case GATE_INTAKING:
                taskList.add(Task.DRIVE);
                taskList.add(Task.WAITING);
                break;
            case SHOOT_END:
            case SHOOT:
                taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case SHOOT_START:
                if (!autoClose) taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case PARK:
                //if (!autoClose || previousObjective != Objective.SHOOT) taskList.add(Task.DRIVE);
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
        }

        currentTask = taskList.get(++taskIndex);

        // set robot state for task
        switch (currentTask) {

            case DRIVE_TO_INTAKE:
                robotAuto.setState(Robot.State.IDLE);
                executeCurrentPath();
                break;
            case DRIVE:
            case DRIVE_TO_SHOOT:
                robotAuto.setState(Robot.State.STORING);
                executeCurrentPath();
                break;
            case INTAKING:
                robotAuto.setState(Robot.State.INTAKING);
                executeCurrentPath();
                break;
            case SHOOTING:
                robotAuto.resetShots();
                robotAuto.setState(Robot.State.SHOOTING);
                break;
            case WAITING:
                if(currentObjective == Objective.GATE_RELEASING) {
                    gateReleasingTimer.start();
                } else if (currentObjective == Objective.GATE_INTAKING) {
                    robotAuto.setState(Robot.State.INTAKING);
                    gateIntakingTimer.start();
                }
                robotAuto.setState(Robot.State.IDLE);
                break;
        }

    }

    private void executeCurrentPath() {
        Key key = new Key(previousObjective, currentTask, currentObjective, autoClose);
        Path path = pathMap.get(key);

        if (true) { //!robotAuto.drivetrainAuto.isFollowerBusy()
            if (path != null){

                switch(currentTask) {
                    case INTAKING:
                        robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.INTAKING_DRIVE_SPEED);
                        robotAuto.drivetrainAuto.followPath(path);
                        break;
                    case DRIVE_TO_SHOOT:
                        robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                        robotAuto.drivetrainAuto.followPathAndHold(path);
                        break;
                    default:
                        robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                        robotAuto.drivetrainAuto.followPath(path);
                        break;
                }

            }else {
                panelsTelemetry.addLine("No path found");
                throw new IllegalArgumentException("Path not defined");
            }
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
            case INTAKING:
                if (robotAuto.drivetrainAuto.isAtEnd())
                    nextTask();
                break;
            case SHOOTING:
                if (robotAuto.getBallsShot() >= 3)
                    nextTask();
                break;
            case WAITING:
                if(currentObjective == Objective.GATE_RELEASING)
                {
                    if (gateReleasingTimer.done()) // timer completed
                        nextTask();
                } else if(currentObjective == Objective.GATE_INTAKING) {
                    if (gateIntakingTimer.done()) // timer completed
                        nextTask();
                } else {
                    // Do Nothing (parked at end)
                }
                break;
        }
    }

    // ----------------------------------------------------
    // PATH MAP
    // ----------------------------------------------------

    private void buildPathMap() {
        // Key-Format: (previousObjective, currentTask, currentObjective, autoClose)

        // =====================================================
        // 1. DRIVE_TO_INTAKE (Start -> Ziel)
        // =====================================================

        // SPIKE_MARK_CLOSE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                AutoPaths.startCloseToIntakingClose);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                AutoPaths.shootCloseToIntakingClose);
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                AutoPaths.startFarToIntakingClose);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                AutoPaths.shootFarToIntakingClose);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                AutoPaths.gateReleasingToIntakingClose);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                AutoPaths.gateReleasingToIntakingClose);

        // SPIKE_MARK_MIDDLE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                AutoPaths.startCloseToIntakingMiddle);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                AutoPaths.shootCloseToIntakingMiddle);
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                AutoPaths.startFarToIntakingMiddle);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                AutoPaths.shootFarToIntakingMiddle);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                AutoPaths.gateReleasingToIntakingMiddle);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                AutoPaths.gateReleasingToIntakingMiddle);

        // SPIKE_MARK_FAR
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                AutoPaths.startCloseToIntakingFar);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                AutoPaths.shootCloseToIntakingFar);
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                AutoPaths.startFarToIntakingFar);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                AutoPaths.shootFarToIntakingFar);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                AutoPaths.gateReleasingToIntakingFar);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                AutoPaths.gateReleasingToIntakingFar);

        // GATE_INTAKING
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE, Objective.GATE_INTAKING, true),
                AutoPaths.startCloseToGateIntaking);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE, Objective.GATE_INTAKING, true),
                AutoPaths.shootCloseToGateIntaking);
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE, Objective.GATE_INTAKING, false),
                AutoPaths.startFarToGateIntaking);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE, Objective.GATE_INTAKING, false),
                AutoPaths.shootFarToGateIntaking);

        // LOADING_ZONE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                AutoPaths.startCloseToLoadingZone);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                AutoPaths.shootCloseToLoadingZone);
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                AutoPaths.startFarToLoadingZone);
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                AutoPaths.shootFarToLoadingZone);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                AutoPaths.gateReleasingToLoadingZone);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                AutoPaths.gateReleasingToLoadingZone);

        // =====================================================
        // 2. INTAKING (Die Aktion vor Ort)
        // =====================================================
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, true), AutoPaths.intakingClose);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, true), AutoPaths.intakingMiddle);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, true), AutoPaths.intakingFar);
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, true), AutoPaths.intakingLoadingZone);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, false), AutoPaths.intakingClose);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, false), AutoPaths.intakingMiddle);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, false), AutoPaths.intakingFar);
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, false), AutoPaths.intakingLoadingZone);

        // =====================================================
        // 3. DRIVE_TO_SHOOT (Zurück zum Schießen)
        // =====================================================

        // FAR Start to Shooting Position
        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, false), AutoPaths.startFarToShootFar);

        // CLOSE Variante (true)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.intakingCloseToShootClose);
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.intakingMiddleToShootClose);
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.intakingFarToShootClose);
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.gateIntakingToShootClose);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.gateReleasingToShootClose);
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), AutoPaths.loadingZoneToShootClose);

        // FAR Variante (false)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.intakingCloseToShootFar);
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.intakingMiddleToShootFar);
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.intakingFarToShootFar);
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.gateIntakingToShootFar);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.gateReleasingToShootFar);
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), AutoPaths.loadingZoneToShootFar);

        // CLOSE End
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.intakingCloseToShootCloseParked);
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.intakingMiddleToShootCloseParked);
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.intakingFarToShootCloseParked);
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.gateIntakingToShootCloseParked);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.gateReleasingToShootCloseParked);
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), AutoPaths.loadingZoneToShootCloseParked);

        // FAR End (same as far normal)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.intakingCloseToShootFar);
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.intakingMiddleToShootFar);
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.intakingFarToShootFar);
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.gateIntakingToShootFar);
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.gateReleasingToShootFar);
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), AutoPaths.loadingZoneToShootFar);

        // =====================================================
        // 4. GATE RELEASING & PARK
        // =====================================================

        // Weg zum Gate nach dem Schießen
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, true), AutoPaths.shootCloseToGateReleasing);
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, false), AutoPaths.shootFarToGateReleasing);
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, true), AutoPaths.startCloseToGateReleasing);
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, false), AutoPaths.shootFarToGateReleasing);

        // Park Close
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, true), AutoPaths.gateIntakingToShootCloseParked); // TODO: add path
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, true), AutoPaths.shootCloseToShootCloseParked);
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, true), AutoPaths.startCloseToShootCloseParked);

        // Park Far
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, false), AutoPaths.gateReleasingToParkedFar);
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, false), AutoPaths.startFarToParkedFar);
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, false), AutoPaths.shootFarToParkedFar);

        // Spezial Park-Shootings (Close)
        //pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), AutoPaths.intakingCloseToShootCloseParked);
        //pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), AutoPaths.intakingMiddleToShootCloseParked);
        //pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), AutoPaths.intakingFarToShootCloseParked);
        //pathMap.put(new Key(Objective.SHOOTING_START, Task.SHOOTING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), AutoPaths.startCloseToShootCloseParked);
    }

    public void updateTelemetry() {
        // Position
        Pose2D botPose = robotAuto.drivetrainAuto.getPose();
        String position = String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.1f",
                botPose.getX(DistanceUnit.INCH),
                botPose.getY(DistanceUnit.INCH),
                botPose.getHeading(AngleUnit.DEGREES));

        // Task / Objective
        panelsTelemetry.addLine("=== Task ===");
        panelsTelemetry.addData("current Objective", currentObjective);
        panelsTelemetry.addData("currentTask", currentTask);
        if (previousObjective != null) panelsTelemetry.addData("previousObjective", previousObjective);

        // Robot States
        panelsTelemetry.addLine("=== ROBOT STATES ===");
        panelsTelemetry.addData("Robot State", robotAuto.getState());
        panelsTelemetry.addData("Intake State", robotAuto.intake.getState());
        panelsTelemetry.addData("Transfer State", robotAuto.transfer.getState());
        panelsTelemetry.addData("Shooter State", robotAuto.shooter.getState());

        // Pose
        panelsTelemetry.addLine("=== POSE ===");
        if (!robotAuto.isUsingLimelight()) {
            panelsTelemetry.addData("Odometry", position);
        } else {
            panelsTelemetry.addData("Limelight", position);
        }
        panelsTelemetry.addData("Distance", robotAuto.drivetrainAuto.getDistance());

        // Shooter Info
        panelsTelemetry.addLine("=== SHOOTER ===");
        if (robotAuto.shooter.getManualOverride()) {
            panelsTelemetry.addLine("Shooter: MANUAL OVERRIDE");
            panelsTelemetry.addData("Hood Position (manual)", robotAuto.shooter.getHoodPositionManual());
            panelsTelemetry.addData("Target Velocity (manual)", robotAuto.shooter.getShooterTargetVelocityManual());
        } else {
            panelsTelemetry.addData("Hood Position", robotAuto.shooter.getHoodPosition());
            panelsTelemetry.addData("Target Velocity", robotAuto.shooter.getShooterTargetVelocity());
        }
        panelsTelemetry.addData("Current Velocity", robotAuto.shooter.getShooterVelocity());

        // Update PanelsTelemetry
        //drawPath();
        drawDebug(robotAuto.drivetrainAuto.getFollower());
        panelsTelemetry.update();
    }
}