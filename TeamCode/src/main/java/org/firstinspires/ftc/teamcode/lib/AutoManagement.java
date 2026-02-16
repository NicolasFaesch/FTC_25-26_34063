package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.List;

public class AutoManagement {


    public enum Objective {
        SHOOTING_START,
        SPIKE_MARK_CLOSE,
        SpikeMarkMiddle,
        SpikeMarkFar,
        GateIntaking,
        GateReleasing,
        LoadingZone,
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

    public boolean autoClose;

    public static List<Task> taskList;

    public static Task currentTask;
    public static int currentTaskIndex;

    public static List<Objective> objectives;
    public static List<Objective> previousObjectives;

    public static Objective previousObjective;
    public static Objective nextObjective;
    public static Objective currentObjective;
    public static int currentObjectiveIndex;

    public void addObjective(Objective objective) {
        objectives.add(objective);
    }

    public void nextObjective() {

        previousObjective = currentObjective;
        previousObjectives.add(previousObjective);
        currentObjective = nextObjective;
        nextObjective = objectives.get(++currentObjectiveIndex+1);


        // build task list
        boolean intakedArtifacts = taskList.contains(Task.INTAKING);
        taskList.clear();
        switch (currentObjective) {
            case SPIKE_MARK_CLOSE:
            case SpikeMarkMiddle:
            case SpikeMarkFar:
            case LoadingZone:
            case GateIntaking:
                taskList.add(Task.DRIVE_TO_INTAKE);
                taskList.add(Task.INTAKING);
                if(nextObjective != Objective.GateReleasing && nextObjective != Objective.PARK) break;
                taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case GateReleasing:
                taskList.add(Task.DRIVE);
                taskList.add(Task.WAITING);
                if (!intakedArtifacts) break;
                taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case SHOOTING_START:
                if(!autoClose) taskList.add(Task.DRIVE_TO_SHOOT);
                taskList.add(Task.SHOOTING);
                break;
            case PARK:
                taskList.add(Task.DRIVE);
                if (autoClose) taskList.add(Task.SHOOTING);
                taskList.add(Task.WAITING);
        }


    }

    public void nextTask() {
        if(currentTaskIndex >= taskList.size()) {
            currentTaskIndex = 0;
            nextObjective();
        } else {
            currentTaskIndex++;
        }
        currentTask = taskList.get(currentTaskIndex);
        switch (currentTask) {
            case DRIVE_TO_INTAKE:
                Path trackingPath;
                switch (currentObjective) {
                    case SPIKE_MARK_CLOSE:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToIntakingClose;
                            else trackingPath = AutoPaths.shootCloseToIntakingClose;
                        } else {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startFarToIntakingClose;
                            else trackingPath = AutoPaths.shootFarToIntakingClose;
                        }
                        break;
                    case SpikeMarkMiddle:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToIntakingMiddle;
                            else trackingPath = AutoPaths.shootCloseToIntakingMiddle;
                        } else {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startFarToIntakingMiddle;
                            else trackingPath = AutoPaths.shootFarToIntakingMiddle;
                        }
                        break;
                    case SpikeMarkFar:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToIntakingFar;
                            else trackingPath = AutoPaths.shootCloseToIntakingFar;
                        } else {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startFarToIntakingFar;
                            else trackingPath = AutoPaths.shootFarToIntakingFar;
                        }
                        break;
                    case GateIntaking:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToGateIntaking;
                            else trackingPath = AutoPaths.shootCloseToGateIntaking;
                        } else {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startFarToGateIntaking;
                            else trackingPath = AutoPaths.shootFarToGateIntaking;
                        }
                        break;
                    case GateReleasing:
                        if(autoClose) {
                            trackingPath = AutoPaths.startCloseToGateReleasing;
                        } else {
                            trackingPath = AutoPaths.startFarToGateReleasing;
                        }
                        break;
                    case LoadingZone:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToLoadingZone;
                            else trackingPath = AutoPaths.shootCloseToLoadingZone;
                        } else {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startFarToLoadingZone;
                            else trackingPath = AutoPaths.shootFarToLoadingZone;
                        }
                        break;
                    }
                robotAuto.drivetrainAuto.followPath(trackingPath);
                break;
            case INTAKING:
                Path intakingPath;
                switch (currentObjective) {
                    case SPIKE_MARK_CLOSE:
                       intakingPath = AutoPaths.intakingClose;
                       break;
                    case SpikeMarkMiddle:
                        intakingPath = AutoPaths.intakingMiddle;
                        break;
                    case SpikeMarkFar:
                        intakingPath = AutoPaths.intakingFar;
                    case LoadingZone:
                        intakingPath = AutoPaths.intakingLoadingZone;
                }
                robotAuto.drivetrainAuto.followPath(intakingPath, INTAKING_DRIVE_SPEED, false);
                break;
            case DRIVE_TO_SHOOT:
                Path trackingPathShoot;
                if (autoClose) {
                    switch (previousObjective) {
                        case SPIKE_MARK_CLOSE:
                            trackingPathShoot = AutoPaths.intakingCloseToShootClose;
                            break;
                        case SpikeMarkMiddle:
                            trackingPathShoot = AutoPaths.intakingMiddleToShootClose;
                            break;
                        case SpikeMarkFar:
                            trackingPathShoot = AutoPaths.intakingFarToShootClose;
                            break;
                        case GateIntaking:
                            trackingPathShoot = AutoPaths.gateIntakingToShootClose;
                            break;
                        case LoadingZone:
                            trackingPathShoot = AutoPaths.loadingZoneToShootClose;
                            break;
                        case GateReleasing:
                            trackingPathShoot = AutoPaths.gateReleasingToShootClose;
                            break;
                        case SHOOTING_START:
                            trackingPathShoot = AutoPaths.startCloseToShootClose;
                    }
                } else {
                    switch (previousObjective) {
                        case SPIKE_MARK_CLOSE:
                            trackingPathShoot = AutoPaths.intakingCloseToShootFar;
                            break;
                        case SpikeMarkMiddle:
                            trackingPathShoot = AutoPaths.intakingMiddleToShootFar;
                            break;
                        case SpikeMarkFar:
                            trackingPathShoot = AutoPaths.intakingFarToShootFar;
                            break;
                        case GateIntaking:
                            trackingPathShoot = AutoPaths.gateIntakingToShootFar;
                            break;
                        case LoadingZone:
                            trackingPathShoot = AutoPaths.loadingZoneToShootFar;
                            break;
                        case GateReleasing:
                            trackingPathShoot = AutoPaths.gateReleasingToShootFar;
                            break;
                        case SHOOTING_START:
                            trackingPathShoot = AutoPaths.startFarToShootFar;
                    }
                }
                robotAuto.drivetrainAuto.followPath(trackingPathShoot);
                break;
            case SHOOTING:
                break;
            case DRIVE:
                Path drivePath;
                if (!autoClose) {
                    if (currentObjective == Objective.PARK) {
                        switch (previousObjective) {
                            case GateReleasing:
                                drivePath = AutoPaths.gateReleasingToParkedFar;
                                break;
                            case SHOOTING_START:
                                drivePath = AutoPaths.startFarToParkedFar;
                                break;
                            default:
                                drivePath = AutoPaths.shootFarToParkedFar;
                                break;
                        }
                    } else if (previousObjective == Objective.GateReleasing) {
                        switch (currentObjective) {
                            case SPIKE_MARK_CLOSE:
                                drivePath = AutoPaths.gateReleasingToIntakingClose;
                                break;
                            case SpikeMarkMiddle:
                                drivePath = AutoPaths.gateReleasingToIntakingMiddle;
                                break;
                            case SpikeMarkFar:
                                drivePath = AutoPaths.gateReleasingToIntakingFar;
                                break;
                            case LoadingZone:
                                drivePath = AutoPaths.gateReleasingToLoadingZone;
                                break;
                        }
                    } else if (nextObjective == Objective.GateReleasing){
                        drivePath = AutoPaths.shootFarToGateReleasing;
                    }

                } else {
                    if (nextObjective == Objective.GateReleasing){
                        switch (currentObjective) {
                            case SPIKE_MARK_CLOSE:
                                drivePath = AutoPaths.intakingCloseToGateReleasing;
                                break;
                            case SpikeMarkMiddle:
                                drivePath = AutoPaths.intakingMiddleToGateReleasing;
                                break;
                            case SpikeMarkFar:
                                drivePath = AutoPaths.intakingFarToGateReleasing;
                                break;
                            case LoadingZone:
                                drivePath = AutoPaths.loadingZoneToGateReleasing;
                                break;
                            default:
                                drivePath = AutoPaths.shootCloseToGateReleasing;
                                break;
                        }

                    } else if (currentObjective == Objective.PARK) {
                        switch (previousObjective) {
                            case SPIKE_MARK_CLOSE:
                                drivePath = AutoPaths.intakingCloseToShootCloseParked;
                                break;
                            case SpikeMarkMiddle:
                                drivePath = AutoPaths.intakingMiddleToShootCloseParked;
                                break;
                            case SpikeMarkFar:
                                drivePath = AutoPaths.intakingFarToShootCloseParked;
                                break;
                            case GateIntaking:
                                drivePath = AutoPaths.gateIntakingToShootCloseParked;
                                break;
                            case LoadingZone:
                                drivePath = AutoPaths.loadingZoneToShootCloseParked;
                                break;
                            case GateReleasing:
                                drivePath = AutoPaths.gateReleasingToShootCloseParked;
                                break;
                            case SHOOTING_START:
                                drivePath = AutoPaths.startCloseToShootCloseParked;
                                break;
                        }
                    }
                }
                robotAuto.drivetrainAuto.followPath(drivePath);
                break;
        }
    }

    public void update() {
        switch(currentTask) {
            case WAITING:
                if(currentObjective == Objective.GateReleasing && Timer stuff){
                nextTask();
                }
                break;
            case DRIVE_TO_INTAKE:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.setState(Robot.State.INTAKING);
                    nextTask();
                }
                break;
            case INTAKING:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    robotAuto.setState(Robot.State.STORING);
                    nextTask();
                }
                break;
            case DRIVE_TO_SHOOT:
                break;
            case SHOOTING:
                break;
            case DRIVE:
                if(!robotAuto.drivetrainAuto.isFollowerBusy()){
                    nextTask();
                }
                break;
        }
    }
}
