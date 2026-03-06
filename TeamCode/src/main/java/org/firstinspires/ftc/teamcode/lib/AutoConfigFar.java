package org.firstinspires.ftc.teamcode.lib;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class AutoConfigFar {
    @Sorter(sort=0)
    public static AutoManagement.Objective FIRST_OBJECTIVE = AutoManagement.Objective.SHOOT_START;
    @Sorter(sort=1)
    public static AutoManagement.Objective SECOND_OBJECTIVE = AutoManagement.Objective.LOADING_ZONE;
    @Sorter(sort=2)
    public static AutoManagement.Objective THIRD_OBJECTIVE = AutoManagement.Objective.SHOOT;
    @Sorter(sort=3)
    public static AutoManagement.Objective FOURTH_OBJECTIVE = AutoManagement.Objective.LOADING_ZONE;
    @Sorter(sort=4)
    public static AutoManagement.Objective FIFTH_OBJECTIVE = AutoManagement.Objective.SHOOT;
    @Sorter(sort=5)
    public static AutoManagement.Objective SIXTH_OBJECTIVE = AutoManagement.Objective.LOADING_ZONE;
    @Sorter(sort=6)
    public static AutoManagement.Objective SEVENTH_OBJECTIVE = AutoManagement.Objective.SHOOT;
    @Sorter(sort=7)
    public static AutoManagement.Objective EIGHTH_OBJECTIVE = AutoManagement.Objective.PARK;
    @Sorter(sort=8)
    public static AutoManagement.Objective NINTH_OBJECTIVE = AutoManagement.Objective.NONE;
    @Sorter(sort=9)
    public static AutoManagement.Objective TENTH_OBJECTIVE = AutoManagement.Objective.NONE;
    @Sorter(sort=10)
    public static AutoManagement.Objective ELEVENTH_OBJECTIVE = AutoManagement.Objective.NONE;
    @Sorter(sort=11)
    public static AutoManagement.Objective TWELVETH_OBJECTIVE = AutoManagement.Objective.NONE;



}