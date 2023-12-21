package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

public class Attachments extends MecanumDrive {

    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx extensionMotor, rotateMotor;
    public Servo claw1Servo, claw2Servo;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_, boolean auto) {

        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize Drivetrain
        initializeDrive(hardwareMap);


        // Motors
        extensionMotor = hardwareMap.get(DcMotorEx.class, names.extension);
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotateMotor = hardwareMap.get(DcMotorEx.class, names.rotate);
        rotateMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Servos
        claw1Servo = hardwareMap.get(Servo.class, names.claw1);
        claw2Servo = hardwareMap.get(Servo.class, names.claw2);


        if (!auto) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


    }


    public void runExtensionMotor(double power) {
        extensionMotor.setPower(power);
    }
    public void runRotateMotor(double power) {
        rotateMotor.setPower(power);
    }

    public void setExtensionMotor(double power, int position) {
        extensionMotor.setPower(power);
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRotateMotor(double power, int position) {
        rotateMotor.setPower(power);
        rotateMotor.setTargetPosition(position);
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void setClaw1Servo(double position) {
        claw1Servo.setPosition(position);
    }
    public void setClaw2Servo(double position) {
        claw2Servo.setPosition(position);
    }

}
