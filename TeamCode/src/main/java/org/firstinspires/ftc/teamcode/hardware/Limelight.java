package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    private static final double LINEAR_VELOCITY_THRESHOLD = 0.05;
    private static final double ANGULAR_VELOCITY_THRESHOLD = 5.0;
    private Limelight3A limelight;
    private LLResult llresult;

    public enum Pipeline {
        RED_GOAL,
        BLUE_GOAL,
        OBELISK
    }

    public Limelight(HardwareMap hardwareMap, Pipeline pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        switchPipeline(pipeline);
        limelight.start();
    }

    public void switchPipeline(Pipeline pipeline) {
        int pipelineNumber = 1;
        switch (pipeline) {
            case RED_GOAL:
                pipelineNumber = 1;
                break;
            case BLUE_GOAL:
                pipelineNumber = 2;
                break;
            case OBELISK:
                pipelineNumber = 0;
                break;
        }
        limelight.pipelineSwitch(pipelineNumber);
    }

    public void update(double heading) {
        limelight.updateRobotOrientation(heading);
    }

    public Pose2D getPose(double velocityX, double velocityY, double angularVelocity) throws Exception {
        double linearVelocity = Math.sqrt(velocityX*velocityX + velocityY*velocityY);

        if(linearVelocity < LINEAR_VELOCITY_THRESHOLD && angularVelocity < ANGULAR_VELOCITY_THRESHOLD) {
            throw new Exception("Robot too quick for limelight pose update.");
        }

        llresult = limelight.getLatestResult();
        if(llresult != null) {
            if(llresult.isValid()) {
                Pose3D botpose3D = llresult.getBotpose();
                return new Pose2D(DistanceUnit.METER, botpose3D.getPosition().x, botpose3D.getPosition().y,
                        AngleUnit.DEGREES, botpose3D.getOrientation().getYaw());
            }
        }

        throw new Exception("invalid limelight result");

    }
}
