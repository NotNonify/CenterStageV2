package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._drive.Localizer;

import java.util.ArrayList;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class CenterstageTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();



    // HANG STUFF
    private boolean releaseToggle = true;
    private double release1Position = Constants.release1Hold;
    private double release2Position = Constants.release2Hold;

    private double hang1Power = 0;
    private double hang2Power = 0;
    private boolean useHangPower = true;
    private boolean hangModeUpdate = false;
    private boolean hangUseEnc = true;
    private int targetHang1Position = 0;
    private int targetHang2Position = 0;
    private int currentHang1Position = 0;
    private int currentHang2Position = 0;


    // CLAW/DRONE STUFF
    private double claw1Position = Constants.clawOpen1;
    private double claw2Position = Constants.clawOpen1;
    private boolean claw1Toggle = true;
    private boolean claw2Toggle = true;

    private boolean droneToggle = true;
    private double dronePosition = Constants.droneHold;


    // MOTOR STUFF
    private double extPower = 0;
    private boolean useExtPower = true;
    private boolean extModeUpdate = false;
    private boolean extUseEnc = true;
    private int targetExtPosition = 0;
    private int currentExtPosition = 0;

    private double rotPower = 0;
    private boolean useRotPower = true;
    private boolean rotModeUpdate = false;
    private boolean rotUseEnc = true;
    private int targetRotPosition = 0;
    private int currentRotPosition = 0;


    // OTHER STUFF
    private int stage = -1;
    private boolean limits = true;


    // FIELD CENTRIC STUFF
    Localizer localizer;
    Pose2d currentPose;
    double heading;
    double fieldCentricOffset;


    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry, false);
        localizer = new Localizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        localizer.setPoseEstimate(PoseStorage.currentPose);
        fieldCentricOffset = PoseStorage.fieldCentricOffset;

        robot.setClaw1Servo(claw1Position);
        robot.setClaw2Servo(claw2Position);
        robot.setRelease1Servo(release1Position);
        robot.setRelease2Servo(release2Position);
        robot.setDroneServo(dronePosition);
    }

    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // Tracking Robot Position using Odo Wheels
        localizer.update();
        currentPose = localizer.getPoseEstimate();
        heading = -currentPose.getHeading();


        double lx = 0;
        double ly = 0;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.6;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.6;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        if (lx != 0 || ly != 0) {
            // Normal D-Pad Drive
            robot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);
        } else {
            // Field Centric Drive
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(heading + fieldCentricOffset);

            robot.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
        }


        /* -------------------------------------------- CHANGE -------------------------------------------- */


        if (gamepad2.b) {
            stage = 0;
        } else if (gamepad2.a) {
            stage = 2;
        }

        if (stage >= -1) {
            switch (stage) {
                // flipping up
                case 0:
                    // bring rot over
                    if (currentRotPosition > Constants.rotSafeExt) {
                        useRotPower = false;
                        targetRotPosition = Constants.rotTeleDrop;
                        rotUseEnc = false;
                    } else {
                        stage++;
                    }
                    break;
                case 1:
                    useRotPower = false;
                    targetExtPosition = Constants.extTeleDrop;
                    rotUseEnc = false;
                    stage = -1;
                    break;

                // flipping down
                case 2:
                    // bring ext down
                    if (currentExtPosition > Constants.extMin + Constants.extTolerance) {
                        useExtPower = false;
                        targetExtPosition = Constants.extMin;
                        extUseEnc = false;
                    } else {
                        stage++;
                    }
                    break;
                case 3:
                    if (currentRotPosition < Constants.rotSlow - Constants.rotTolerance) {
                        useRotPower = false;
                        targetRotPosition = Constants.rotSlow;
                        rotUseEnc = false;
                    } else {
                        stage++;
                    }
                    break;
                case 4:
                    if (currentRotPosition < Constants.rotDrive) {
                        useRotPower = false;
                        targetRotPosition += Constants.rotSpeed;
                        rotUseEnc = false;
                    } else {
                        stage = -1;
                    }
                    break;
            }
        }



        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            limits = !limits;
        }



        if (gamepad1.a && !previousGamepad1.a) {
            droneToggle = !droneToggle;
        }

        if (droneToggle) {
            dronePosition = Constants.droneHold;
        } else {
            dronePosition = Constants.droneRelease;
        }


        if (gamepad1.right_trigger > 0.2 && !(previousGamepad1.right_trigger > 0.2)) {
            releaseToggle = !releaseToggle;
        }

        if (releaseToggle) {
            release1Position = Constants.release1Hold;
            release2Position = Constants.release2Hold;
        } else {
            release1Position = Constants.release1Open;
            release2Position = Constants.release2Open;
        }


        if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            claw1Toggle = !claw1Toggle;
        }
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            claw2Toggle = !claw2Toggle;
        }

        if (claw1Toggle) {
            claw1Position = Constants.clawOpen1;
        } else {
            claw1Position = Constants.clawClose1;
        }

        if (claw2Toggle) {
            claw2Position = Constants.clawOpen2;
        } else {
            claw2Position = Constants.clawClose2;
        }


        /*

        // Hold Extension motor if power is 0
        if (useExtPower && extPower == 0) {
            useExtPower = false;
            targetExtPosition = currentExtPosition;
            extUseEnc = true;
        }

        // Hold Rotate motor if power is 0
        if (useRotPower && rotPower == 0) {
            useRotPower = false;
            targetRotPosition = currentRotPosition;
            rotUseEnc = true;
        }

        */



        if (gamepad1.y) {
            hangUseEnc = true;

            if (currentHang1Position < Constants.hangMax || !limits) {
                useHangPower = true;
                hang1Power = 1;
            }
            if (currentHang2Position < Constants.hangMax || !limits) {
                useHangPower = true;
                hang2Power = 1;
            }
        } else if (gamepad1.x) {
            hangUseEnc = true;

            if (currentHang1Position > Constants.hangMin || !limits) {
                useHangPower = true;
                hang1Power = -1;
            }
            if (currentHang2Position > Constants.hangMin || !limits) {
                useHangPower = true;
                hang2Power = -1;
            }
        } else {
            useHangPower = false;
            targetHang1Position = currentHang1Position;
            targetHang2Position = currentHang2Position;
            hangUseEnc = true;
        }


        double rotJoystick = gamepad2.left_stick_x;
        if (rotJoystick > 0.12) {
            rotUseEnc = true;
            // user trying to rotate right
            if (currentRotPosition > Constants.rotMax || !limits) {
                useRotPower = true;
                rotPower = rotJoystick * Constants.rotUpRatio * -1;
            } else {
                rotPower = 0;
            }
        } else if (rotJoystick < -0.12) {
            rotUseEnc = true;
            // user trying to rotate left
            if (currentRotPosition < Constants.rotMin || !limits) {
                useRotPower = true;
                rotPower = rotJoystick * Constants.rotDownRatio * -1;
                if (currentRotPosition > Constants.rotSlow) {
                    rotPower *= Constants.rotSlowRatio;
                }
            } else {
                rotPower = 0;
            }
        } else if (useRotPower) {
            useRotPower = false;
            targetRotPosition = currentRotPosition;
            rotUseEnc = true;
        }


        double extJoystick = -gamepad2.left_stick_y;
        if (extJoystick > 0.12) {
            extUseEnc = true;
            // user trying to extend out
            if (currentExtPosition < Constants.extMax || !limits) {
                useExtPower = true;
                extPower = extJoystick * Constants.extUpRatio;
            } else {
                extPower = 0;
            }
        } else if (extJoystick < -0.12) {
            extUseEnc = true;
            // user trying to extend back
            if (currentExtPosition > Constants.extMin || !limits) {
                useExtPower = true;
                extPower = extJoystick * Constants.extDownRatio;
                if (currentExtPosition > Constants.extSlow) {
                    extPower *= Constants.extSlowRatio;
                }
            } else {
                extPower = 0;
            }
        } else if (useExtPower) {
            useExtPower = false;
            targetExtPosition = currentExtPosition;
            extUseEnc = true;
        }




        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

        currentExtPosition = robot.getExtMotorPosition();
        currentRotPosition = robot.getRotMotorPosition();
        currentHang1Position = robot.getHang1Position();
        currentHang2Position = robot.getHang2Position();



        /* -------------------------------------------- ACTION -------------------------------------------- */


        robot.setClaw1Servo(claw1Position);
        robot.setClaw2Servo(claw2Position);
        robot.setRelease1Servo(release1Position);
        robot.setRelease2Servo(release2Position);
        robot.setDroneServo(dronePosition);


        if (rotModeUpdate && rotUseEnc) {
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotModeUpdate = false;
        }

        if (extModeUpdate && extUseEnc) {
            robot.extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extModeUpdate = false;
        }

        if (hangModeUpdate && hangUseEnc) {
            robot.hang1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hang2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangModeUpdate = false;
        }


        if (useExtPower) {
            robot.runExtensionMotor(extPower);
        } else {
            setExt(targetExtPosition);
        }

        if (useRotPower) {
            robot.runRotateMotor(rotPower);
        } else {
            setRot(targetRotPosition);
        }

        if (useHangPower) {
            robot.runHang1Motor(hang1Power);
            robot.runHang1Motor(hang2Power);
        } else {
            setHang1(targetHang1Position);
            setHang2(targetHang2Position);
        }


        /* -------------------------------------------- TELEMETRY -------------------------------------------- */

        telemetry.addData("Ext Joy", extJoystick);
        telemetry.addData("Rot Joy", rotJoystick);
        telemetry.addLine();
        telemetry.addData("Ext Pos", currentExtPosition);
        telemetry.addData("Rot Pos", currentRotPosition);
        telemetry.addData("Hang1 Pos", currentHang1Position);
        telemetry.addData("Hang2 Pos", currentHang2Position);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Claw 1 Toggle", claw1Toggle);
        telemetry.addData("Claw 2 Toggle", claw2Toggle);
        telemetry.addData("Release Toggle", releaseToggle);
        telemetry.addData("Drone Toggle", droneToggle);
        telemetry.addLine();
        telemetry.addData("Claw 1 Pos", claw1Position);
        telemetry.addData("Claw 2 Pos", claw2Position);
        telemetry.addData("Release 1 Pos", release1Position);
        telemetry.addData("Release 2 Pos", release2Position);
        telemetry.addData("Drone Pos", dronePosition);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Limits", limits);
        telemetry.update();

    }


    void setHang1(int position) {
        robot.setHang1Motor(1, position);
        hangUseEnc = false;
        hangModeUpdate = true;
    }

    void setHang2(int position) {
        robot.setHang2Motor(1, position);
        hangUseEnc = false;
        hangModeUpdate = true;
    }


    void setExt(int position) {
        robot.setExtensionMotor(1, position);
        extUseEnc = false;
        extModeUpdate = true;
    }

    void setRot(int position) {
        robot.setRotateMotor(1, position);
        rotUseEnc = false;
        rotModeUpdate = true;
    }

}
