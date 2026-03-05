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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.ColorLED;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

import java.util.*;
import java.util.concurrent.TimeUnit;

public class AutoManagement {

    // Feld zum Zeichnen
    //private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
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
        PARK,
        NONE
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
    Map<Key, List<Path>> pathMap = new HashMap<>();
    private Map<Key, Integer> pathIndexMap = new HashMap<>();

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
                //if (!autoClose) taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case PARK:
                //if (!autoClose || previousObjective != Objective.SHOOT) taskList.add(Task.DRIVE);
                taskList.add(Task.WAITING);
                break;
            case NONE:
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
                robotAuto.colorLED.setColor(ColorLED.Color.BLUE);
                robotAuto.setState(Robot.State.IDLE);
                executeCurrentPath();
                break;
            case DRIVE:
            case DRIVE_TO_SHOOT:
                robotAuto.colorLED.setColor(ColorLED.Color.BLUE);
                robotAuto.setState(Robot.State.STORING);
                executeCurrentPath();
                break;
            case INTAKING:
                robotAuto.colorLED.setColor(ColorLED.Color.PURPLE);
                robotAuto.setState(Robot.State.INTAKING);
                executeCurrentPath();
                break;
            case SHOOTING:
                robotAuto.colorLED.setColor(ColorLED.Color.GREEN);
                robotAuto.resetShots();
                robotAuto.setState(Robot.State.SHOOTING);
                break;
            case WAITING:
                robotAuto.colorLED.setColor(ColorLED.Color.ORANGE);
                if(currentObjective == Objective.GATE_RELEASING) {
                    gateReleasingTimer.start();
                    robotAuto.setState(Robot.State.IDLE);
                } else if (currentObjective == Objective.GATE_INTAKING) {
                    robotAuto.setState(Robot.State.INTAKING);
                    gateIntakingTimer.start();
                } else {
                    robotAuto.setState(Robot.State.IDLE);
                }
                break;
        }

    }

    private void executeCurrentPath() {

        Key key = new Key(previousObjective, currentTask, currentObjective, autoClose);
        List<Path> paths = pathMap.get(key);

        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path not defined for key: " + key);
        }

        int index = pathIndexMap.getOrDefault(key, 0);

        if (index >= paths.size()) {
            return; // alle Paths abgefahren
        }

        Path currentPath = paths.get(index);

        switch (currentTask) {

            case INTAKING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.INTAKING_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPath(currentPath);
                break;

            case DRIVE_TO_SHOOT:
            case WAITING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPathAndHold(currentPath);
                break;

            default:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPath(currentPath);
                break;
        }
    }
    /*
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
                    case WAITING:
                        robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                        robotAuto.drivetrainAuto.followPathAndHold(path);
                    default:
                        robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                        robotAuto.drivetrainAuto.followPath(path);
                        break;
                }

            }else {
                //panelsTelemetry.addLine("No path found");
                throw new IllegalArgumentException("Path not defined");
            }
        }
    }

    private void executeCurrentPath() {
        Key key = new Key(previousObjective, currentTask, currentObjective, autoClose);
        List<Path> paths = pathMap.get(key);

        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path not defined for key: " + key);
        }

        Path path = paths.get(0);

        switch (currentTask) {

            case INTAKING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.INTAKING_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPath(path);
                break;

            case DRIVE_TO_SHOOT:
            case WAITING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPathAndHold(path);
                break;

            default:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPath(path);
                break;
        }
    }*/

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

                if (robotAuto.drivetrainAuto.isAtEnd()) {

                    Key key = new Key(previousObjective, currentTask, currentObjective, autoClose);
                    List<Path> paths = pathMap.get(key);

                    int index = pathIndexMap.getOrDefault(key, 0) + 1;

                    if (paths != null && index < paths.size()) {
                        pathIndexMap.put(key, index);
                        executeCurrentPath(); // nächsten Path starten
                    } else {
                        pathIndexMap.remove(key); // reset für später
                        nextTask(); // Task komplett fertig
                    }
                }
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

        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, true),
                List.of(AutoPaths.startCloseToShootClose));

        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, false),
                List.of(AutoPaths.startFarToShootFar));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true),
                List.of(AutoPaths.startCloseToShootClose));

        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false),
                List.of(AutoPaths.startFarToShootFar));
        // =====================================================
        // 1. DRIVE_TO_INTAKE (Start -> Ziel)
        // =====================================================

        // SPIKE_MARK_CLOSE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                List.of(AutoPaths.startCloseToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                List.of(AutoPaths.shootCloseToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                List.of(AutoPaths.startFarToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                List.of(AutoPaths.shootFarToIntakingClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                List.of(AutoPaths.gateReleasingToIntakingClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                List.of(AutoPaths.gateReleasingToIntakingClose));

        // SPIKE_MARK_MIDDLE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                List.of(AutoPaths.startCloseToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                List.of(AutoPaths.shootCloseToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                List.of(AutoPaths.startFarToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                List.of(AutoPaths.shootFarToIntakingMiddle));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                List.of(AutoPaths.gateReleasingToIntakingMiddle));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                List.of(AutoPaths.gateReleasingToIntakingMiddle));

        // SPIKE_MARK_FAR
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                List.of(AutoPaths.startCloseToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                List.of(AutoPaths.shootCloseToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                List.of(AutoPaths.startFarToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                List.of(AutoPaths.shootFarToIntakingFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                List.of(AutoPaths.gateReleasingToIntakingFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                List.of(AutoPaths.gateReleasingToIntakingFar));

        // GATE_INTAKING
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE, Objective.GATE_INTAKING, true),
                List.of(AutoPaths.startCloseToGateIntaking));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE, Objective.GATE_INTAKING, true),
                List.of(AutoPaths.shootCloseToGateIntaking));
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE, Objective.GATE_INTAKING, false),
                List.of(AutoPaths.startFarToGateIntaking));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE, Objective.GATE_INTAKING, false),
                List.of(AutoPaths.shootFarToGateIntaking));

        // LOADING_ZONE
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                List.of(AutoPaths.startCloseToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                List.of(AutoPaths.shootCloseToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT_START,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                List.of(AutoPaths.startFarToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT,Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                List.of(AutoPaths.shootFarToLoadingZone));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                List.of(AutoPaths.gateReleasingToLoadingZone));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                List.of(AutoPaths.gateReleasingToLoadingZone));

        // =====================================================
        // 2. INTAKING (Die Aktion vor Ort)
        // =====================================================
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, true), List.of(AutoPaths.intakingClose));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, true), List.of(AutoPaths.intakingMiddle));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, true), List.of(AutoPaths.intakingFar));
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, true), AutoPaths.loadingZoneChain);
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, false), List.of(AutoPaths.intakingClose));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, false), List.of(AutoPaths.intakingMiddle));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, false), List.of(AutoPaths.intakingFar));
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, false), AutoPaths.loadingZoneChain);

        // =====================================================
        // 3. DRIVE_TO_SHOOT (Zurück zum Schießen)
        // =====================================================

        // FAR Start to Shooting Position
        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, false), List.of(AutoPaths.startFarToShootFar));

        // CLOSE Variante (true)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.intakingCloseToShootClose));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.intakingMiddleToShootClose));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.intakingFarToShootClose));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.gateIntakingToShootClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.gateReleasingToShootClose));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), List.of(AutoPaths.loadingZoneToShootClose));

        // FAR Variante (false)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.intakingCloseToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.intakingMiddleToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.intakingFarToShootFar));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.gateIntakingToShootFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.gateReleasingToShootFar));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), List.of(AutoPaths.loadingZoneToShootFar));

        // CLOSE End
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.intakingCloseToShootCloseParked));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.intakingMiddleToShootCloseParked));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.intakingFarToShootCloseParked));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.gateIntakingToShootCloseParked));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.gateReleasingToShootCloseParked));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), List.of(AutoPaths.loadingZoneToShootCloseParked));

        // FAR End (same as far normal)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.intakingCloseToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.intakingMiddleToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.intakingFarToShootFar));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.gateIntakingToShootFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.gateReleasingToShootFar));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), List.of(AutoPaths.loadingZoneToShootFar));

        // =====================================================
        // 4. GATE RELEASING & PARK
        // =====================================================

        // Weg zum Gate nach dem Schießen
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.shootCloseToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.shootFarToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.startCloseToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.shootFarToGateReleasing));

        // Park Close
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, true), List.of(AutoPaths.gateIntakingToShootCloseParked)); // TODO: add path
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, true), List.of(AutoPaths.shootCloseToShootCloseParked));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, true), List.of(AutoPaths.startCloseToShootCloseParked));

        // Park Far
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, false), List.of(AutoPaths.gateReleasingToParkedFar));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, false), List.of(AutoPaths.startFarToParkedFar));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, false), List.of(AutoPaths.shootFarToParkedFar));

        // Spezial Park-Shootings (Close)
        //pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), List.of(AutoPaths.intakingCloseToShootCloseParked));
        //pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), List.of(AutoPaths.intakingMiddleToShootCloseParked));
        //pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.INTAKING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), List.of(AutoPaths.intakingFarToShootCloseParked));
        //pathMap.put(new Key(Objective.SHOOTING_START, Task.SHOOTING, Task.DRIVE_TO_SHOOT, Objective.PARK, true), List.of(AutoPaths.startCloseToShootCloseParked));



        // Intaking to Gate Releasing
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.intakingCloseToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.intakingMiddleToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.intakingFarToGateReleasing));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE, Objective.GATE_RELEASING, true), List.of(AutoPaths.loadingZoneToGateReleasing));

        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.intakingCloseToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.intakingMiddleToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.intakingFarToGateReleasing));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE, Objective.GATE_RELEASING, false), List.of(AutoPaths.loadingZoneToGateReleasing));
    }

    public void updateTelemetry(TelemetryManager panelsTelemetry, Telemetry telemetry) {
        // Task / Objective
        panelsTelemetry.addLine("=== Task ===");
        panelsTelemetry.addData("current Objective", currentObjective);
        panelsTelemetry.addData("currentTask", currentTask);
        if (previousObjective != null) panelsTelemetry.addData("previousObjective", previousObjective);

        telemetry.addLine("=== Task ===");
        telemetry.addData("current Objective", currentObjective);
        telemetry.addData("currentTask", currentTask);
        if (previousObjective != null) telemetry.addData("previousObjective", previousObjective);


        robotAuto.updateTelemetry(panelsTelemetry, telemetry);
    }
}