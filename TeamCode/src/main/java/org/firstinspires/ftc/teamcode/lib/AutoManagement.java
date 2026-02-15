package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.paths.Path;

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
                if(nextObjective != Objective.GateReleasing) break;
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
                if (!autoClose) taskList.add(Task.DRIVE);
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
                            trackingPath = AutoPaths.shootFarToIntakingClose;
                        }
                        break;
                    case SpikeMarkMiddle:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToIntakingMiddle;
                            else trackingPath = AutoPaths.shootCloseToIntakingMiddle;
                        } else {
                            trackingPath = AutoPaths.shootFarToIntakingMiddle;
                        }
                        break;
                    case SpikeMarkFar:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToIntakingFar;
                            else trackingPath = AutoPaths.shootCloseToIntakingFar;
                        } else {
                            trackingPath = AutoPaths.shootFarToIntakingFar;
                        }
                        break;
                    case GateIntaking:
                        if(autoClose) {
                            if (previousObjective == Objective.SHOOTING_START)
                                trackingPath = AutoPaths.startCloseToGateIntaking;
                            else trackingPath = AutoPaths.shootCloseToGateIntaking;
                        } else {
                            trackingPath = AutoPaths.shootFarToGateIntaking;
                        }
                        break;
                    case LoadingZone:
                        if(autoClose) {
                            trackingPath = AutoPaths.shootCloseToLoadingZone;
                        } else {
                            trackingPath = AutoPaths.shootFarToLoadingZone;
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
                }
                robotAuto.drivetrainAuto.followPath(intakingPath, INTAKING_DRIVE_SPEED, false);
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
