package org.firstinspires.ftc.teamcode.Autonoumous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousMethods;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PoseConstants;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode._trajectorysequence.TrajectorySequence;

@Autonomous(name="Auto_BlueBackdrop", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_BlueBackdrop extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {

        PoseStorage.fieldCentricOffset = Math.toRadians(-90);
        int propLocation = 2;

        robot.initialize(hardwareMap, telemetry, true);

        while (!isStarted()) {;
            propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }
        int finalPropLocation = propLocation;
        telemetry.addLine("Final Location: " + finalPropLocation);
        telemetry.update();

        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(PoseConstants.blueBackdrop_start)
                        .addDisplacementMarker(() -> {
                            setArm(Constants.rotAutoDrop, Constants.extAutoDrop);
                        })
                        .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            setClaw(Constants.clawOpen1, Constants.clawClose2);
                            sleep(400);
                            setArm(Constants.rotMin, Constants.extAutoDrop);
                        })
                        .lineToSplineHeading(new Pose2d(30, 36, Math.toRadians(180)))
                        .addDisplacementMarker(() -> {
                            setClaw(Constants.clawOpen1, Constants.clawOpen2);
                        })
                        .build()
        );

        PoseStorage.currentPose = robot.getPoseEstimate();

    }
}
