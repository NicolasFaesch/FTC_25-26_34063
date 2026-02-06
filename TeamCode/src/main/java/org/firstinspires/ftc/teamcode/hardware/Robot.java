package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Controller;

public class Robot {
    public Limelight limelight;
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;
    public ColorLED colorLED;

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
        STORING
    }

    protected State state;

    private boolean usingLimelight = false;


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        limelight = new Limelight(hardwareMap,
                (alliance == Alliance.RED) ? Limelight.Pipeline.RED_GOAL: Limelight.Pipeline.BLUE_GOAL);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.ShooterMotorIdlingState.OFF);
        colorLED = new ColorLED(hardwareMap);

        state = State.IDLE;

    }

    protected void update(Pose2D currentPose) {
        //intake.update();
        //transfer.update();

        // Set transfer to feeding mode
        if(shooter.getState() == Shooter.State.AIMING || shooter.getState() == Shooter.State.SHOOTING) {
            if(shooter.getFeederState() != Shooter.FeederState.READY) {
                transfer.setState(Transfer.State.IDLE);
            } else {
                transfer.setState(Transfer.State.FEEDING);
            }
        }

        // Update limelight
        limelight.update(currentPose.getHeading(AngleUnit.DEGREES));

    }

    public void setState(State state) {
        if(state != this.state) {
            this.state = state;
            switch(state) {
                case IDLE:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.OFF);
                    intake.setState(Intake.State.IDLE);
                    transfer.setState(Transfer.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case INTAKING:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.OFF);
                    intake.setState(Intake.State.INTAKING);
                    transfer.setState(Transfer.State.INTAKING);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case OUTTAKING:
                    intake.setState(Intake.State.OUTTAKING);
                    transfer.setState(Transfer.State.OUTTAKING);
                    shooter.setState(Shooter.State.IDLE);
                    break;
                case SHOOTING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    shooter.setState(Shooter.State.SHOOTING);
                    break;
                case AIMING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.FEEDING);
                    shooter.setState(Shooter.State.AIMING);
                    break;
                case STORING:
                    shooter.setShooterMotorIdlingMode(Shooter.ShooterMotorIdlingState.SPINNING);
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.FEEDING);
                    shooter.setState(Shooter.State.IDLE);
                    break;
            }
        }

    }

    public State getState() {
        return state;
    }

    protected Pose2D getLimelightPose(double xVelocity, double yVelocity, double headingVelocity, boolean useMT2) {
        Pose2D limelightPose;
        if(!useMT2) { // use MT1 for calib
            limelightPose = limelight.getPose(xVelocity, yVelocity, headingVelocity);
        } else { // otherwise MT2
            limelightPose = limelight.getPoseMT2(xVelocity, yVelocity, headingVelocity);
        }
        usingLimelight = (limelightPose != null);
        return limelightPose;
    }

    public boolean isUsingLimelight() {return usingLimelight;}

    public void resetShots() {shooter.resetShots();}

    public int getBallsShot() {return shooter.getBallsShot();}




}
