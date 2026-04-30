package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;

import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.field.PanelsField;
import com.bylazar.field.FieldManager;
import com.bylazar.field.Style;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
            if (currentTask == Task.INTAKING) {
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

    // Updated to use PathChain instead of List<Path>
    Map<Key, PathChain> pathMap = new HashMap<>();

    private int objectiveIndex = 0;
    private int taskIndex = 0;

    private Objective previousObjective;
    private Objective currentObjective;

    private Task currentTask;

    private Timing.Timer gateReleasingTimer = new Timing.Timer(AutoConstants.GATE_RELEASING_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer gateIntakingTimer = new Timing.Timer(AutoConstants.GATE_INTAKING_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer shootingTimer = new Timing.Timer(AutoConstants.SHOOTING_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer intakeClearingTimer = new Timing.Timer(AutoConstants.INTAKE_CLEARING_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer loadingZoneTimer = new Timing.Timer(AutoConstants.LOADING_ZONE_TIME, TimeUnit.MILLISECONDS);

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
                taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case PARK:
                if(previousObjective != Objective.SHOOT_END)
                    taskList.add(Task.DRIVE);
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
            case DRIVE:
            case DRIVE_TO_INTAKE:
                robotAuto.colorLED.setColor(ColorLED.Color.WHITE);
                robotAuto.setState(Robot.State.INTAKING);
                executeCurrentPath();
                break;
            case DRIVE_TO_SHOOT:
                robotAuto.colorLED.setColor(ColorLED.Color.GREEN);
                if(currentObjective == Objective.SHOOT_START) {
                    robotAuto.setState(Robot.State.INTAKING);
                } else {
                    robotAuto.setState(Robot.State.OUTTAKING);
                }
                executeCurrentPath();
                intakeClearingTimer.start();
                break;
            case INTAKING:
                robotAuto.colorLED.setColor(ColorLED.Color.BLUE);
                robotAuto.setState(Robot.State.INTAKING);
                if(currentObjective == Objective.LOADING_ZONE)
                    loadingZoneTimer.start();
                executeCurrentPath();
                break;
            case SHOOTING:
                robotAuto.colorLED.setColor(ColorLED.Color.PURPLE);
                robotAuto.setState(Robot.State.SHOOTING);
                break;
            case WAITING:
                robotAuto.colorLED.setColor(ColorLED.Color.ORANGE);
                if (currentObjective == Objective.GATE_RELEASING) {
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

        // Retrieve PathChain instead of List<Path>
        PathChain currentChain = pathMap.get(key);

        if (currentChain == null) {
            throw new IllegalArgumentException("Path not defined for key: " + key);
        }

        switch (currentTask) {
            case INTAKING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.INTAKING_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPathChain(currentChain);
                break;

            case DRIVE_TO_SHOOT:
            case WAITING:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPathChainAndHold(currentChain);
                break;

            default:
                robotAuto.drivetrainAuto.setFollowerMaxPower(AutoConstants.REGULAR_DRIVE_SPEED);
                robotAuto.drivetrainAuto.followPathChain(currentChain);
                break;
        }
    }

    // ----------------------------------------------------
    // UPDATE LOOP
    // ----------------------------------------------------

    public void update() {
        robotAuto.drivetrainAuto.update();

        PoseFile.writePose(robotAuto.drivetrainAuto.getPose());

        if (currentTask == null)
            return;

        switch (currentTask) {
            case DRIVE_TO_SHOOT:
                if (robotAuto.getState() == Robot.State.OUTTAKING) {
                    if (intakeClearingTimer.done()) {
                        robotAuto.setState(Robot.State.INTAKING);
                        intakeClearingTimer.start();
                    }
                } else if(robotAuto.getState() == Robot.State.INTAKING){
                    if (intakeClearingTimer.done()) {
                        shootingTimer.start();
                        robotAuto.setState(Robot.State.SHOOTING);
                    }
                } else {
                    if (!robotAuto.shooter.getReadyToShoot()) {
                        shootingTimer.pause();
                    } else if (!shootingTimer.isTimerOn()) {
                        shootingTimer.resume();
                    }
                }
            case DRIVE:
            case DRIVE_TO_INTAKE:
            case INTAKING:
                // Pedro Pathing handles PathChains automatically, just check for the end
                if (robotAuto.drivetrainAuto.isAtEnd()) {
                    nextTask();
                }
                if(currentObjective == Objective.LOADING_ZONE && loadingZoneTimer.done())
                    nextTask();
                break;
            case SHOOTING:
                if (!robotAuto.shooter.getReadyToShoot()) {
                    shootingTimer.pause();
                } else if (!shootingTimer.isTimerOn()) {
                    shootingTimer.resume();
                }
                if (shootingTimer.done())
                    nextTask();
                break;
            case WAITING:
                if (currentObjective == Objective.GATE_RELEASING) {
                    if (gateReleasingTimer.done())
                        nextTask();
                } else if (currentObjective == Objective.GATE_INTAKING) {
                    if (gateIntakingTimer.done())
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
                new PathChain(AutoPaths.startCloseToShootClose));

        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, false),
                new PathChain(AutoPaths.startFarToShootFar));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true),
                new PathChain(AutoPaths.startCloseToShootClose));

        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false),
                new PathChain(AutoPaths.startFarToShootFar));

        // =====================================================
        // 1. DRIVE_TO_INTAKE (Start -> Ziel)
        // =====================================================

        // SPIKE_MARK_CLOSE
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                new PathChain(AutoPaths.startCloseToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                new PathChain(AutoPaths.shootCloseToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                new PathChain(AutoPaths.startFarToIntakingClose));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                new PathChain(AutoPaths.shootFarToIntakingClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, true),
                new PathChain(AutoPaths.gateReleasingToIntakingClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_CLOSE, false),
                new PathChain(AutoPaths.gateReleasingToIntakingClose));

        // SPIKE_MARK_MIDDLE
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                new PathChain(AutoPaths.startCloseToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                new PathChain(AutoPaths.shootCloseToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                new PathChain(AutoPaths.startFarToIntakingMiddle));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                new PathChain(AutoPaths.shootFarToIntakingMiddle));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, true),
                new PathChain(AutoPaths.gateReleasingToIntakingMiddle));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_MIDDLE, false),
                new PathChain(AutoPaths.gateReleasingToIntakingMiddle));

        // SPIKE_MARK_FAR
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                new PathChain(AutoPaths.startCloseToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                new PathChain(AutoPaths.shootCloseToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                new PathChain(AutoPaths.startFarToIntakingFar));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                new PathChain(AutoPaths.shootFarToIntakingFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, true),
                new PathChain(AutoPaths.gateReleasingToIntakingFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.SPIKE_MARK_FAR, false),
                new PathChain(AutoPaths.gateReleasingToIntakingFar));

        // GATE_INTAKING
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_INTAKING, true),
                new PathChain(AutoPaths.startCloseToGateIntaking, AutoPaths.gateIntakingShuffleTo));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_INTAKING, true),
                new PathChain(AutoPaths.shootCloseToGateIntaking, AutoPaths.gateIntakingShuffleTo));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_INTAKING, false),
                new PathChain(AutoPaths.startFarToGateIntaking, AutoPaths.gateIntakingShuffleTo));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_INTAKING, false),
                new PathChain(AutoPaths.shootFarToGateIntaking, AutoPaths.gateIntakingShuffleTo));


        // LOADING_ZONE
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                new PathChain(AutoPaths.startCloseToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                new PathChain(AutoPaths.shootCloseToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                new PathChain(AutoPaths.startFarToLoadingZone));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                new PathChain(AutoPaths.shootFarToLoadingZone));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, true),
                new PathChain(AutoPaths.gateReleasingToLoadingZone));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_INTAKE, Objective.LOADING_ZONE, false),
                new PathChain(AutoPaths.gateReleasingToLoadingZone));

        // =====================================================
        // 2. INTAKING (Die Aktion vor Ort)
        // =====================================================
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, true), new PathChain(AutoPaths.intakingClose));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, true), new PathChain(AutoPaths.intakingMiddle));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, true), new PathChain(AutoPaths.intakingFar));
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, true), new PathChain(AutoPaths.loadingZoneChain.get(0)));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_CLOSE, false), new PathChain(AutoPaths.intakingClose));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_MIDDLE, false), new PathChain(AutoPaths.intakingMiddle));
        pathMap.put(new Key(null, Task.INTAKING, Objective.SPIKE_MARK_FAR, false), new PathChain(AutoPaths.intakingFar));
        pathMap.put(new Key(null, Task.INTAKING, Objective.LOADING_ZONE, false), new PathChain(AutoPaths.loadingZoneChain.get(0)));

        // =====================================================
        // 3. DRIVE_TO_SHOOT (Zurück zum Schießen)
        // =====================================================

        // FAR Start to Shooting Position
        pathMap.put(new Key(null, Task.DRIVE_TO_SHOOT, Objective.SHOOT_START, false), new PathChain(AutoPaths.startFarToShootFar));

        // CLOSE Variante (true)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.intakingCloseToShootClose));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.intakingMiddleToShootClose));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.intakingFarToShootClose));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.gateIntakingToShootClose));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.gateReleasingToShootClose));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, true), new PathChain(AutoPaths.loadingZoneToShootClose));

        // FAR Variante (false)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.intakingCloseToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.intakingMiddleToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.intakingFarToShootFar));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.gateIntakingToShootFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.gateReleasingToShootFar));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT, false), new PathChain(AutoPaths.loadingZoneToShootFar));

        // CLOSE End
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.intakingCloseToShootCloseParked));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.intakingMiddleToShootCloseParked));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.intakingFarToShootCloseParked));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.gateIntakingToShootCloseParked));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.gateReleasingToShootCloseParked));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, true), new PathChain(AutoPaths.loadingZoneToShootCloseParked));

        // FAR End (same as far normal)
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.intakingCloseToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.intakingMiddleToShootFar));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.intakingFarToShootFar));
        pathMap.put(new Key(Objective.GATE_INTAKING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.gateIntakingToShootFar));
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.gateReleasingToShootFar));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE_TO_SHOOT, Objective.SHOOT_END, false), new PathChain(AutoPaths.loadingZoneToShootFar));

        // =====================================================
        // 4. GATE RELEASING & PARK
        // =====================================================

        // Weg zum Gate nach dem Schießen
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.shootCloseToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.shootFarToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.startCloseToGateReleasing));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.shootFarToGateReleasing));

        // Park Close
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.gateIntakingToShootCloseParked)); // TODO: add path
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.shootCloseToShootCloseParked));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.startCloseToShootCloseParked));

        // Park Far
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, false), new PathChain(AutoPaths.gateReleasingToParkedFar));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, false), new PathChain(AutoPaths.startFarToParkedFar));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, false), new PathChain(AutoPaths.shootFarToParkedFar));

        // Park Close Releasing
        pathMap.put(new Key(Objective.GATE_RELEASING, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.gateReleasingToParkedClose));
        pathMap.put(new Key(Objective.SHOOT, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.startCloseToParkedClose));
        pathMap.put(new Key(Objective.SHOOT_START, Task.DRIVE, Objective.PARK, true), new PathChain(AutoPaths.shootCloseToParkedClose));

        // Intaking to Gate Releasing
        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.intakingCloseToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.intakingMiddleToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.intakingFarToGateReleasing));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE, Objective.GATE_RELEASING, true), new PathChain(AutoPaths.loadingZoneToGateReleasing));

        pathMap.put(new Key(Objective.SPIKE_MARK_CLOSE, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.intakingCloseToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_MIDDLE, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.intakingMiddleToGateReleasing));
        pathMap.put(new Key(Objective.SPIKE_MARK_FAR, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.intakingFarToGateReleasing));
        pathMap.put(new Key(Objective.LOADING_ZONE, Task.DRIVE, Objective.GATE_RELEASING, false), new PathChain(AutoPaths.loadingZoneToGateReleasing));
    }

    public void updateTelemetry(TelemetryManager panelsTelemetry, Telemetry telemetry) {
        // Task / Objective
        panelsTelemetry.addLine("=== Task ===");
        panelsTelemetry.addData("current Objective", currentObjective);
        panelsTelemetry.addData("currentTask", currentTask);
        if (previousObjective != null)
            panelsTelemetry.addData("previousObjective", previousObjective);

        telemetry.addLine("=== Task ===");
        telemetry.addData("current Objective", currentObjective);
        telemetry.addData("currentTask", currentTask);
        if (previousObjective != null) telemetry.addData("previousObjective", previousObjective);

        Pose2D savedPose = PoseFile.readPose();

        telemetry.addData("Loaded Pose", savedPose);

        robotAuto.updateTelemetry(panelsTelemetry, telemetry);
    }
}