package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.DynamicAiming;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.PositionChecker;
import org.firstinspires.ftc.teamcode.lib.ShooterLUT;

public class Robot {
    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public ColorLED colorLED;
    public Turret turret;

    public enum Alliance {
        RED,
        BLUE
    }

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        SHOOTING,
        AIMING,
        STORING,
        PARKING
    }

    protected State state;
    protected Alliance alliance;

    protected boolean validShootingPose;
    protected boolean turretReady;
    protected boolean blockerDisengaged;


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        colorLED = new ColorLED(hardwareMap);
        turret = new Turret(hardwareMap);

        this.alliance = alliance;

        validShootingPose = false;
        turretReady = false;
        blockerDisengaged = false;

        DynamicAiming.setTargetPose(this.alliance == Alliance.RED ? PoseStorage.targetPoseRed : PoseStorage.targetPoseBlue);
        DynamicAiming.createLUTs();

        state = null; // force state change initially
        setState(State.IDLE);
    }

    protected void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    protected void update() {
        blockerDisengaged = shooter.getBlockerState() == Shooter.BlockerState.DISENGAGED;
        DynamicAiming.AimingParams aimingParams = DynamicAiming.calculateTargeting(drivetrain.getPose(), drivetrain.getVelocityX(), drivetrain.getVelocityY(), drivetrain.getAngularVelocity());

        intake.update(blockerDisengaged);
        transfer.update(blockerDisengaged);
        turret.update(0);//aimingParams.turretAngle);

        validShootingPose = PositionChecker.checkInZones(drivetrain.getPose()) && DynamicAiming.getTargetDistance() > ShooterLUT.minDistance;
        turretReady = turret.isOnTarget();

        shooter.update(state != State.PARKING ? aimingParams.hoodAngle : Shooter.HOOD_MIN_POSITION, aimingParams.flywheelRpm, true, true);//validShootingPose, turretReady);
    }


    public void setState(State state) {
        if (state != this.state) {
            this.state = state;
            switch (state) {
                case IDLE:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.OFF);
                    intake.setState(Intake.State.IDLE);
                    transfer.setState(Transfer.State.IDLE);
                    turret.setState(Turret.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case INTAKING:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.OFF);
                    intake.setState(Intake.State.INTAKING);
                    transfer.setState(Transfer.State.INTAKING);
                    turret.setState(Turret.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case OUTTAKING:
                    intake.setState(Intake.State.OUTTAKING);
                    transfer.setState(Transfer.State.OUTTAKING);
                    turret.setState(Turret.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case SHOOTING:
                    intake.setState(Intake.State.FEEDING);
                    transfer.setState(Transfer.State.FEEDING);
                    turret.setState(Turret.State.TRACKING);
                    shooter.setState(Shooter.State.SHOOTING);
                    break;
                case AIMING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    turret.setState(Turret.State.TRACKING);
                    shooter.setState(Shooter.State.AIMING);
                    break;
                case STORING:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.SPINNING);
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    turret.setState(Turret.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case PARKING:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.OFF);
                    intake.setState(Intake.State.IDLE);
                    transfer.setState(Transfer.State.IDLE);
                    turret.setState(Turret.State.STORED);
                    shooter.setState(Shooter.State.IDLE);
                    break;
            }
        }

    }

    public State getState() {
        return state;
    }
}
