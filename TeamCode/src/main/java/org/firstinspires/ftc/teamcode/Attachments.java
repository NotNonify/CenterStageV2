package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode._drive.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Attachments extends MecanumDrive {

    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx extensionMotor, rotateMotor;
    public DcMotorEx hang1, hang2;
    public Servo release1, release2;
    public Servo claw1Servo, claw2Servo, drone;


    public WebcamName webcam1, webcam2;
    public VisionProcessor visionProcessor;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

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

        hang1 = hardwareMap.get(DcMotorEx.class, names.hang1);
        hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang2 = hardwareMap.get(DcMotorEx.class, names.hang2);
        hang2.setDirection(DcMotorEx.Direction.REVERSE);
        hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Servos
        claw1Servo = hardwareMap.get(Servo.class, names.claw1); // Upper Claw
        claw2Servo = hardwareMap.get(Servo.class, names.claw2); // Lower Claw

        release1 = hardwareMap.get(Servo.class, names.release1);
        release2 = hardwareMap.get(Servo.class, names.release2);

        drone = hardwareMap.get(Servo.class, names.drone);



        // Camera
//        webcam1 = hardwareMap.get(WebcamName.class, names.webcam1);
//        webcam2 = hardwareMap.get(WebcamName.class, names.webcam2);


        if (!auto) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            visionProcessor = new VisionProcessor();
            // visionPortal = VisionPortal.easyCreateWithDefaults(webcam, visionProcessor);
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            aprilTagProcessor.setDecimation(2);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(visionProcessor)
                    .build();
        }
    }


    public void runExtensionMotor(double power) {
        extensionMotor.setPower(power);
    }
    public void runRotateMotor(double power) {
        rotateMotor.setPower(power);
    }

    public void runHang1Motor(double power) {
        hang1.setPower(power);
    }
    public void runHang2Motor(double power) {
        hang2.setPower(power);
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

    public void setHang1Motor(double power, int position) {
        hang1.setPower(power);
        hang1.setTargetPosition(position);
        hang1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setHang2Motor(double power, int position) {
        hang2.setPower(power);
        hang2.setTargetPosition(position);
        hang2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void setClaw1Servo(double position) {
        claw1Servo.setPosition(position);
    }
    public void setClaw2Servo(double position) {
        claw2Servo.setPosition(position);
    }

    public void setRelease1Servo(double position) {
        release1.setPosition(position);
    }
    public void setRelease2Servo(double position) {
        release2.setPosition(position);
    }

    public void setDroneServo(double position) {
        drone.setPosition(position);
    }

    public int getRotMotorPosition() {
        return rotateMotor.getCurrentPosition();
    }
    public int getExtMotorPosition() {
        return extensionMotor.getCurrentPosition();
    }
    public int getHang1Position() {
        return hang1.getCurrentPosition();
    }
    public int getHang2Position() {
        return hang2.getCurrentPosition();
    }

    public int getPropLocation() {
        VisionProcessor.Selected selection = visionProcessor.getSelection();
        if (selection == VisionProcessor.Selected.LEFT) {
            return 1;
        } else if (selection == VisionProcessor.Selected.MIDDLE) {
            return 2;
        } else if (selection == VisionProcessor.Selected.RIGHT) {
            return 3;
        } else {
            return 3;
        }
    }

}
