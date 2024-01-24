package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    /* -------------------------------------------- AUTO CONSTANTS -------------------------------------------- */

    public static int parkLoc = 2;
    public static boolean continueAutoAfterSpikeMark = true;
    public static int testPropLoc = 1;



    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */

    public static double moveSpeed = 1;
    public static double rotateSpeed = 1;


    public static double slowerMoveVel = 10;
    public static double slowerSplineVel = 30;


    /* -------------------------------------------- SERVO CONSTANTS -------------------------------------------- */

    public static double clawClose1 = 0.5;
    public static double clawOpen1 = 0.4;
    public static double clawClose2 = 0.4;
    public static double clawOpen2 = 0.53;

    public static double release1Hold = 0;
    public static double release2Hold = 0;
    public static double release1Open = 1;
    public static double release2Open = 1;

    public static double droneRelease = 0;
    public static double droneHold = 0;


    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */

    public static int rotMin = 0;
    public static int rotMax = -1200;
    public static double rotUpRatio = 1;
    public static double rotDownRatio = 0.7;
    public static int rotSlow = 200;
    public static double rotSlowRatio = 0.5;
    public static int rotTolerance = 50;
    public static int rotSpeed = 5;

    public static int rotMed = -200;
    public static int rotSafeExt = -900;
    public static int rotDrive = -50;
    public static int rotAutoDrop = -1000;
    public static int rotTeleDrop = -1000;


    public static int extMin = 0;
    public static int extMax = 380;
    public static double extUpRatio = 1;
    public static double extDownRatio = 0.7;
    public static int extSlow = 100;
    public static double extSlowRatio = 0.5;
    public static int extTolerance = 10;

    public static int extAutoDrop = 100;
    public static int extTeleDrop = 100;



    public static int hangMin = 0;
    public static int hangMax = 500;






    /* -------------------------------------------- VISION RECTANGLE CONSTANTS -------------------------------------------- */

    public static int rLx = 5;
    public static int rLy = 360;
    public static int rLw = 100;
    public static int rLh = 60;

    public static int rRx = 500;
    public static int rRy = 360;
    public static int rRw = 135;
    public static int rRh = 80;

    public static int rMx = 140;
    public static int rMy = 360;
    public static int rMw = 300;
    public static int rMh = 60;

    /* --------------------------------------------  APRILTAG CONSTANTS ------------------------------------------ */
    public static double DESIRED_DISTANCE = 7;
    public static double DROP_TOP_FIRST = 1.8;
    public static double CAMERA_TO_CENTER = 8;
    public static double SPEED_GAIN  =  0.02;
    public static double STRAFE_GAIN =  0.015;
    public static double TURN_GAIN   =  0.01;
    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_STRAFE = 0.1;
    public static double MAX_AUTO_TURN  = 0.05;




}
