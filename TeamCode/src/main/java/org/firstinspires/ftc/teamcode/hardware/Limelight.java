package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.concurrent.TimeUnit;

@Configurable
public class Limelight {
    public static double LINEAR_VELOCITY_THRESHOLD = 0.05;
    public static double ANGULAR_VELOCITY_THRESHOLD = 5.0;

    public static double MAX_DISTANCE_MT1 = 2.0; //in meters

    public static long UPDATE_PERIOD_MT1 = 300; //in milliseconds

    private Limelight3A limelight;
    private LLResult llresult;

    private Timing.Timer timerMT1 = new Timing.Timer(UPDATE_PERIOD_MT1, TimeUnit.MILLISECONDS);

    public enum Pipeline {
        RED_GOAL,
        BLUE_GOAL,
        OBELISK
    }

    public Limelight(HardwareMap hardwareMap, Pipeline pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        switchPipeline(pipeline);
        limelight.start();
        timerMT1.start();
    }

    public void switchPipeline(Pipeline pipeline) {
        int pipelineNumber = 1;
        switch (pipeline) {
            case RED_GOAL:
                pipelineNumber = 1;
                break;
            case BLUE_GOAL:
                pipelineNumber = 2; //TODO: Check if pipeline is correct.
                break;
            case OBELISK:
                pipelineNumber = 0;
                break;
        }
        limelight.pipelineSwitch(pipelineNumber);
    }

    public void update(double heading) {
        limelight.updateRobotOrientation(heading);
        llresult = limelight.getLatestResult();
    }

    public Pose2D getPose(double velocityX, double velocityY, double angularVelocity) {
        double linearVelocity = Math.sqrt(velocityX*velocityX + velocityY*velocityY);

        if(linearVelocity > LINEAR_VELOCITY_THRESHOLD || Math.abs(angularVelocity) > ANGULAR_VELOCITY_THRESHOLD) {
            return null;
        }

        llresult = limelight.getLatestResult();
        if(llresult != null) {
            if(llresult.isValid()) {
                Pose3D botpose3D = llresult.getBotpose();
                return new Pose2D(DistanceUnit.METER, botpose3D.getPosition().x, botpose3D.getPosition().y,
                        AngleUnit.DEGREES, botpose3D.getOrientation().getYaw());
            }
        }

        return null;

    }

    public Pose2D getPoseMT2(double velocityX, double velocityY, double angularVelocity) {
        double linearVelocity = Math.sqrt(velocityX*velocityX + velocityY*velocityY);

        if(linearVelocity > LINEAR_VELOCITY_THRESHOLD || Math.abs(angularVelocity) > ANGULAR_VELOCITY_THRESHOLD) {
            return null;
        }

        llresult = limelight.getLatestResult();
        if(llresult != null) {
            if(llresult.isValid()) {

                Pose3D botpose3D;
                if (llresult.getBotposeAvgDist() < MAX_DISTANCE_MT1 && timerMT1.done()) {
                    botpose3D = new Pose3D(llresult.getBotpose_MT2().getPosition(), llresult.getBotpose().getOrientation());
                    timerMT1.start();
                } else {
                    botpose3D = llresult.getBotpose_MT2();
                }
                return new Pose2D(DistanceUnit.METER, botpose3D.getPosition().x, botpose3D.getPosition().y,
                        AngleUnit.DEGREES, botpose3D.getOrientation().getYaw());
            }
        }

        return null;

    }
}
