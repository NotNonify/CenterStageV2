package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

public abstract class AutonomousMethods extends LinearOpMode {

    public Attachments robot = new Attachments();

    public ElapsedTime runtime = new ElapsedTime();

    public boolean opModeStatus() {
        return opModeIsActive();
    }

    public void setArm(int rot, int ext) {
        robot.setExtensionMotor(1, Constants.extMin);

        // wait before rotating until extention is close to the min
        while (robot.getExtMotorPosition() > Constants.extMin + Constants.extTolerance) {
            robot.update();
        }
        robot.setRotateMotor(1, rot);

        // wait before extending depending on which direction to rotate
        if (robot.getRotMotorPosition() < rot) {
            while (robot.getRotMotorPosition() < rot - Constants.rotTolerance) {
                robot.update();
            }
        } else {
            while (robot.getRotMotorPosition() > Constants.rotSafeExt - Constants.rotTolerance) {
                robot.update();
            }
        }

        robot.setExtensionMotor(1, ext);
    }

    public void setClaw(double c1, double c2) {
        robot.setClaw1Servo(c1);
        robot.setClaw2Servo(c2);
    }


    /*
     * Sleeps for x amount of milliseconds while updating the roadrunner position
     */
    public void roadrunnerSleep(int milliseconds) {
        long timeStamp = runtime.now(TimeUnit.MILLISECONDS);
        while (runtime.now(TimeUnit.MILLISECONDS) - timeStamp <= milliseconds) {
            robot.update();
        }
    }

}
