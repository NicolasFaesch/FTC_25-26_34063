package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    protected Limelight limelight;
    protected Intake intake;
    protected Shooter shooter;
    protected Transfer transfer;

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


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        limelight = new Limelight(hardwareMap,
                (alliance == Alliance.RED) ? Limelight.Pipeline.RED_GOAL: Limelight.Pipeline.BLUE_GOAL);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap, Shooter.ShooterMotorIdlingState.OFF);

        state = State.IDLE;

    }

    protected void update() {
        //intake.update();
        //transfer.update();
    }

    public void setState(State state) {
        if(state != this.state) {
            this.state = state;
            switch(state) {
                case IDLE:
                    intake.setState(Intake.State.IDLE);
                    transfer.setState(Transfer.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                case INTAKING:
                    intake.setState(Intake.State.INTAKING);
                    transfer.setState(Transfer.State.INTAKING);
                    shooter.setState(Shooter.State.IDLE);
                case OUTTAKING:
                    intake.setState(Intake.State.OUTTAKING);
                    transfer.setState(Transfer.State.OUTTAKING);
                    shooter.setState(Shooter.State.IDLE);
                case SHOOTING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    shooter.setState(Shooter.State.SHOOTING);
                case AIMING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    shooter.setState(Shooter.State.AIMING);
                case STORING:
                    intake.setState(Intake.State.STORING);
                    transfer.setState(Transfer.State.STORING);
                    shooter.setState(Shooter.State.IDLE);
            }
        }

    }

    public State getState() {
        return state;
    }




}
