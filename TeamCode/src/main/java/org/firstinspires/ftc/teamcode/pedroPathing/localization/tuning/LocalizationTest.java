package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.FlippTele;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config

@TeleOp(group = "Pedro Pathing Tuning", name = "Localization Test")
public class LocalizationTest extends OpMode {
    private PIDController controller;
    private PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int slidestarget = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public double wrist_at;
    public double armPose = 0;
    public AnalogInput wristencoder;

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;

    double mode = 1;
    double county = 2;
    public double basketmove = 1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1800;
    double armbasket = 1725;
    double twistbasket = .5;
    double wristbasket = .2;
    double slidespecimen = .5;
    double armspecimen = 1380;
    double wristspecimen = .3;
    double twistspecimen = .5;
    double armspecimenpickup = 60;
    double wristspecimenpickup = .51;
    double xpress = 1;
    double times = 0;
    public Button1 buttons1 = null;
    public Button2 buttons2 = null;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    public double flippose = .561;
    double rpress = 1;

    public RevTouchSensor limitfront;
    public RevTouchSensor limitfront2;
    public DigitalChannel limitwrist1;
    public double r1press = 1;
    double press = 1;
    double slidesPose = 0;
    double offset = 0;
    double many = 1;
    double newpos = -312;
    double ypress = 1;
    double check = 1;
    double oldangle = 0;
    double oldtarget = 0;
    double dpress = 1;
    public spin gripspinny;
    public DigitalChannel limitwrist3;
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_4,         // Then we're gonna wait a second
        TRAJECTORY_5,         // Finally, we're gonna turn again
        TRAJECTORY_6,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    double front = 0;
    public double dropping = 1;
    public double special_pick = 1;
    double scored = 1;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    double just = 0;
    boolean ready = false;
    double aapress = 1;
    ElapsedTime drivetime = new ElapsedTime();
    double ticks = .002866;
    double conversion = ticks * armticks;
    public double inta = 1;
    public flippy flip;
    double wristpose = .5;
    double twistpose = 0;
    double lasttraj = 0;
    double safety = 1;
    double flipsafe = 1;
    boolean auto = false;
    private Path score3;
    private Path score3ish;
    private Path score4;
    private Path score4ish;
    private Path score5;
    private Path score5ish;
    private Path comeback1;
    private Path comeback1ish;
    public double forward = 5;
    public double start_auto = 1;
    public double a = 1;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        gripspinny = new spin();
        gripspinny.initialize();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        limitwrist3 = hardwareMap.get(DigitalChannel.class, "limitwrist3");
        limitwrist1 = hardwareMap.get(DigitalChannel.class, "limitwrist1");
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        limitfront2 = hardwareMap.get(RevTouchSensor.class, "limitfront2");
        wristencoder = hardwareMap.get(AnalogInput.class, "wristencoder");
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1.setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        buttons1 = new Button1();
        buttons2 = new Button2();
        flip = new flippy();
        flip.initialize();
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        buttons2.button();
        arm();
        drop();
        spit();
        extra_in();
        basket();
        dropoff();
        eddy();
        basketdrop();
        spit2();
        check();
        safeflip();
        servosafe();
        buttons1.button();
        release();
        poseUpdater.update();
        dashboardPoseTracker.update();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        telemetryA.addData("x", poseUpdater.getPose().getX() + 4.1);
        telemetryA.addData("y", poseUpdater.getPose().getY() - 16.9 );
        telemetryA.addData("heading", Math.toDegrees(poseUpdater.getPose().getHeading()));
        telemetryA.addData("total heading", Math.toDegrees(poseUpdater.getTotalHeading()));
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    public void arm(){
        toplimit = 1406 + (2 * slideticks * 2);
        wrist_at = abs(1 - wristencoder.getVoltage() / 3.3);
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = -slides.getCurrentPosition() / slideticks * .03 / 19.6;
        armf = .001 + -slides.getCurrentPosition() / slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        if (-250 < slidesPose - slidestarget && slidesPose - slidestarget < 250 && ((gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1))&& !auto) {
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (!gamepad2.ps && otherPose < bottomlimit && gamepad2.right_stick_y > 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else if (slidesPose > toplimit && gamepad2.right_stick_y < 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else {
                slides.setPower(gamepad2.right_stick_y * .75);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }
        } else {
            slides.setPower(-power);
        }
        armcontroller.setPID(armp, armi, armd);
        armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if (abs(abs(armPose) - abs(armtarget)) < 150 && ((gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) && !auto) && buttons2.lastbutton != "a") {
            if (armPose < newpos + 45 && gamepad2.left_stick_y > 0) {
                Arm1.setPower(0);
                Arm2.setPower(0);
                armtarget = (int) (armPose);
            } else if(dpress == 2){
                Arm1.setPower(-armpower);
                Arm2.setPower(-armpower);
            }else {
                Arm1.setPower(gamepad2.left_stick_y / 2);
                Arm2.setPower(gamepad2.left_stick_y / 2);
                armtarget = (int) (armPose);
            }
        } else {
            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
        }
        telemetry.addData("pose", armPose);
        telemetry.addData("target", armtarget);
        telemetry.addData("power", -armpower);
        telemetry.addData("now - target", slidesPose - slidestarget);
        telemetry.addData("pose", slidesPose);
        telemetry.addData("target", slidestarget);
        telemetry.addData("power", -power);
        telemetry.addData("Limit1", limitwrist1.getState());
        telemetry.addData("Limit2", limitwrist3.getState());
        telemetry.addData("Flippy position", flip.getPosition());
        telemetry.addData("x", poseUpdater.getPose().getX() + 27);
        telemetry.addData("y", poseUpdater.getPose().getY() + 16);
        telemetry.addData("heading", Math.toDegrees(poseUpdater.getPose().getHeading()));
        telemetry.addData("total heading", Math.toDegrees(poseUpdater.getTotalHeading()));
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        telemetry.update();
        if(a == 2) {
            wristy.setPosition(wristpose);
            twisty.setPosition(twistpose + .019);
            flip.setPosition(flippose);
        }
    }
    public class Button2{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad2.a) {
                    button = "a";
                }
                else if(gamepad2.b){
                    button = "b";
                }
                else if (gamepad2.x){
                    button = "x";
                }
                else if (gamepad2.y){
                    button = "y";
                }
                else if (gamepad2.right_bumper){
                    button = "r1";
                }
                else if (gamepad2.left_bumper){
                    button = "l1";
                }
                else if (gamepad2.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad2.right_trigger > .4){
                    button = "r2";
                }else if (gamepad2.dpad_up){
                    button = "up";
                }
                else if (gamepad2.dpad_down){
                    button = "down";
                }
                else if (gamepad2.dpad_left){
                    button = "left";
                }
                else if (gamepad2.dpad_right){
                    button = "right";
                }else if (gamepad2.ps){
                    button = "ps";
                }
                else if (gamepad2.left_stick_button){
                    button = "l3";
                }
                else if (gamepad2.right_stick_button){
                    button = "r3";
                }else if (gamepad2.left_stick_y > .6){
                    button = "lsy";
                }else if (gamepad2.left_stick_y < -.6){
                    button = "lsyu";
                }
            }
            endbutton();
            ButtonControl();
        }
        public void ButtonControl(){
            if(nowbutton == "ps"){
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slidestarget = 0;
                nowbutton = "";
            }
            if(nowbutton == "l2"){
                apress = 2;
                lastbutton = "";
                nowbutton = "";
                dpress = 1;
            }
            if(nowbutton == "r2"){
                a = 2;
                wristpose = .5;
                twistpose = 0;
                armtarget = 0;
                slidestarget = 0;
                flippose = .561;
                safety = 2;
                gripspinny.setPower(0);
                lastbutton = "";
                nowbutton = "";
                dpress = 1;
            }
            if(lastbutton == ""){
                if (nowbutton == "l3") {
                    armtarget = 2022;
                    wristpose = .34;
                    twistpose = 0;
                    slidestarget = 20;
                    flippose = .583;
                    lastbutton = "l3";
                    nowbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "r3"){
                    armtarget = (int) 1742;
                    wristpose = .5;
                    flippose = .561;
                    basketmove = 2;
                    twistpose = 0;
                    lastbutton = "r3";
                    nowbutton = "";
                    dpress = 1;
                }
                if(dpress == 3 && nowbutton == "a"){
                    slidestarget = (int) oldtarget;
                    armtarget = 0;
                    flippose = .613;
                    wristpose = .293;
                    nowbutton = "";
                    lastbutton = "a";
                    twistpose = 0;
                    gripspinny.setPower(-1);
                    dpress = 1;
                }
                else if(nowbutton == "a"){
                    //Arm goes out
                    armtarget = 0;
                    a = 2;
                    slidestarget = (int) (2 * slideticks * 2);
                    wristpose = .293;
                    twistpose = 0;
                    flippose = .613;
                    gripspinny.setPower(-1);
                    lastbutton = "a";
                    nowbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "b"){
                    //Spit out
                    ypress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                    dpress = 1;
                }
                else if (nowbutton == "l1"){
                    //Arm moves to hang specimen
                    armtarget = 732;
                    slidestarget = 648;
                    a = 2;
                    wristpose = .69;
                    twistpose = 0;
                    flippose = .651;
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "r1"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    a = 2;
                    flipsafe = 2;
                    gripspinny.setPower(-1);
                    lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "y"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    twistpose = 0;
                    a = 2;
                    gripspinny.setPower(-1);
                    lastbutton = "y";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "l3"){
                if(nowbutton == "l3"){
                    armtarget = 0;
                    lastbutton = "";
                    nowbutton = "";
                }

            }
            else if(lastbutton == "y"){
                if(nowbutton == "y" || !limitwrist1.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = (int) 1742;
                    wristpose = .5;
                    flippose = .561;
                    basketmove = 2;
                    twistpose = 0;
                    lastbutton = "r3";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1" || !limitwrist1.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = 732;
                    slidestarget = 648;
                    wristpose = .69;
                    twistpose = 0;
                    flippose = .651;
                    gripspinny.setPower(-1);
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "r3"){
                if(nowbutton == "r3") {
                    flippose = .5;
                    wristpose = .6;
                    rpress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "a"){
                if(!limitwrist1.getState() || !limitwrist3.getState()){
                    //brings arm back
                    wristpose = .281;
                    twistpose = 0;
                    armtarget = 0;
                    oldtarget = slidesPose;
                    slidestarget = 0;
                    flippose = .561;
                    dpress = 2;
                    aapress = 2;
                    nowbutton = "";
                    lastbutton = "";
                } else if(nowbutton == "a"){
                    //brings arm back
                    wristpose = .281;
                    twistpose = 0;
                    armtarget = 0;
                    oldtarget = slidesPose;
                    slidestarget = 0;
                    flippose = .561;
                    aapress = 2;
                    nowbutton = "";
                    lastbutton = "";
                }
                if(nowbutton == "lsy"){
                    flippose = .692;
                    armtarget = 0;
                    wristpose = .293;
                    nowbutton = "";
                }
                if(nowbutton == "lsyu"){
                    wristpose = .281;
                    flippose = .613;
                    nowbutton = "";
                }
                else if(nowbutton == "right"){
                    //twists right
                    twistpose = 0;
                    nowbutton = "";
                }
                else if (nowbutton == "left"){
                    //twists left
                    twistpose = .28;
                    nowbutton = "";
                }
                else if(nowbutton == "b"){
                    //reverse intake
                    gripspinny.setPower(gripspinny.getPower() * -1);
                    nowbutton = "";
                }

            }

            else if(lastbutton == "l1"){
                if (nowbutton == "l1" || (limitfront.isPressed() || limitfront2.isPressed())) {
                    //Arm drops block on the hang and goes back in
                    wristpose = .5;
                    slidestarget = 0;
                    gripspinny.setPower(1);
                    lastbutton = "";
                    nowbutton = "";

                }

            }


        }
        public void endbutton(){
            if (!gamepad2.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad2.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad2.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad2.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad2.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad2.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad2.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad2.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad2.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad2.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad2.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad2.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad2.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad2.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad2.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            } else if (gamepad2.left_stick_y > .6){
                nowbutton = "lsy";
                button = "";
            }else if (gamepad2.left_stick_y < -.6){
                nowbutton = "lsyu";
                button = "";
            }
        }
    }
    public class Button1{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad1.a) {
                    button = "a";
                }
                else if(gamepad1.b){
                    button = "b";
                }
                else if (gamepad1.x){
                    button = "x";
                }
                else if (gamepad1.y){
                    button = "y";
                }
                else if (gamepad1.right_bumper){
                    button = "r1";
                }
                else if (gamepad1.left_bumper){
                    button = "l1";
                }
                else if (gamepad1.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad1.right_trigger > .4){
                    button = "r2";
                }else if (gamepad1.dpad_up){
                    button = "up";
                }
                else if (gamepad1.dpad_down){
                    button = "down";
                }
                else if (gamepad1.dpad_left){
                    button = "left";
                }
                else if (gamepad1.dpad_right){
                    button = "right";
                }else if (gamepad1.ps){
                    button = "ps";
                }
                else if (gamepad1.left_stick_button){
                    button = "l3";
                }
                else if (gamepad1.right_stick_button){
                    button = "r3";
                }
            }
            endbutton();
            ButtonControl();
        }
        public void ButtonControl(){
            if(lastbutton == "") {
                if (nowbutton == "r2") {
                    press = 2;
                    nowbutton = "";
                    lastbutton = "";
                    dpress = 1;
                }

                else if (nowbutton == "r1") {
                    slidestarget = 548;
                    wristpose = .633;
                    twistpose = 0;
                    flippose = .796;
                    armtarget = 0;
                    gripspinny.setPower(-1);
                    nowbutton = "";
                    lastbutton = "r1";
                }/* else if (nowbutton == "l2") {
                    //Arm moves to pick up stuff from eddy
                    armtarget = 732;
                    wristpose = .43;
                    slidestarget = 0;
                    flippose = .025;
                    flipsafe = 2;
                    gripspinny.setPower(-1);
                    start_auto = 2;
                    lastbutton = "l2";
                    nowbutton = "";
                    dpress = 1;*/
            }/*else if(lastbutton == "l2"){
                if(nowbutton == "l2"){
                    auto = !auto;
                    follower.breakFollowing();
                    nowbutton = "";
                    lastbutton = "";
                }
            }*/
            else if(lastbutton == "r1"){
                if(nowbutton == "r1" || !limitwrist1.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = 0;
                    slidestarget = 20;
                    twistpose = 0;
                    wristpose = .281;
                    flippose = .561;
                    gripspinny.setPower(0);
                    dpress = 1;
                    lastbutton = "";
                    nowbutton = "";

                }
            }/*

            else if(lastbutton == "l1"){
                if(!limitwrist1.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_3;
                    drive.followTrajectorySequenceAsync(trajectory3);
                    drivetime = new ElapsedTime();
                    drive.setPoseEstimate(startPose);
                    gripspinny.setPower(0);
                    lastbutton = "";
                    nowbutton = "";

                }
                if(nowbutton == "r2"){
                    armtarget = 0;
                    slidestarget = 0;
                    gripspinny.setPower(0);
                    wristpose = 0;
                    nowbutton = "";
                    lastbutton = "";
                }*/

        }
        public void endbutton(){
            if (!gamepad1.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad1.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad1.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad1.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad1.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad1.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad1.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad1.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad1.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad1.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad1.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad1.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad1.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad1.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad1.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            }
        }
    }
    public void check(){
        if(dpress == 2 && slidesPose < 100){
            if(!limitwrist1.getState() || !limitwrist3.getState()){
                dpress = 1;
            }else{
                dpress = 3;
            }
        }
    }
    public void extra_in(){
        if(r1press == 2){
            runtime = new ElapsedTime();
            r1press = 3;
        }else if(r1press == 3 && runtime.time(TimeUnit.MILLISECONDS) < 200){
            gripspinny.setPower(-1);
        }else if(r1press == 3){
            gripspinny.setPower(0);
            r1press = 1;
        }
    }
    public void servosafe(){
        if(safety == 2 && abs(flip.getPosition() - flippose) < .04){
            wristpose = .281;
            safety = 1;
        }
    }
    public void safeflip(){
        if(flipsafe == 2 && flip.getPosition() < .45){
            twistpose = .56;
            flipsafe = 1;
        }
    }
    public void drop(){
        if(xpress == 1.5){
            runtime = new ElapsedTime();
            xpress = 2;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 750){
            gripspinny.setPower(0);
            xpress = 1;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 250){
            gripspinny.setPower(1);
        }
    }
    public void basket(){
        if(basketmove == 2 && abs(armtarget - armPose) < 50){
            slidestarget = 1608;
            basketmove = 1;
        }
    }

    public void spit(){
        if(apress == 2){
            runtime = new ElapsedTime();
            apress = 3;
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 275){
            gripspinny.setPower(1);
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 650){
            gripspinny.setPower(-1);
        }else if(apress == 3){
            gripspinny.setPower(0);
            apress = 1;
        }
    }
    public void spit2() {
        if (aapress == 2) {
            runtime = new ElapsedTime();
            aapress = 3;
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 100) {
            gripspinny.setPower(1);
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500) {
            gripspinny.setPower(-1);
        } else if (aapress == 3) {
            gripspinny.setPower(0);
            aapress = 1;
        }
    }
    public void dropoff(){
        if(ypress == 1.5){
            runtime = new ElapsedTime();
            gripspinny.setPower(1);
            ypress = 2;
        }else if(ypress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 200){
            gripspinny.setPower(0);
            ypress = 1;
        }

    }
    public void basketdrop(){
        if(rpress == 1.5){
            if(abs(flip.getPosition() - flippose) < .01){
                rpress = 1.75;
            }
        } else if(rpress == 1.75){
            runtime = new ElapsedTime();
            gripspinny.setPower(.25);
            rpress = 2;
        }else if(rpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 500){
            gripspinny.setPower(0);
            flippose = .561;
            wristpose = .281;
            rpress = 2.5;
        }else if(rpress == 2.5){
            if(abs(wrist_at - wristpose) < .04){
                rpress = 3;
            }
        } else if(rpress == 3){
            slidestarget = 0;
            armtarget = 0;
            rpress = 1;
        }

    }
    public void eddy(){
        if(press == 2){
            armtarget = 0;
            slidestarget = 1608;
            wristpose = .221;
            flippose = .571;
            twistpose = 0;
            press = 3;
        }else if(press == 3 && abs(abs(slidesPose) - abs(slidestarget)) < 700){
            ypress = 1.5;
            press = 4;
        }else if(press == 4 && ypress == 1){
            armtarget = 0;
            slidestarget = 0;
            flippose = .561;
            wristpose = .281;
            press = 1;
        }
    }
    public class spin{
        public CRServo spinny1;
        public CRServo spinny2;
        public void initialize(){
            spinny1 = hardwareMap.get(CRServo.class, "spinny1");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinny2.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void setPower(double power){
            spinny1.setPower(power);
            spinny2.setPower(power);
        }
        public double getPower(){
            return spinny1.getPower();
        }
    }


    public class flippy{
        public Servo flippy1;
        public Servo flippy2;
        AnalogInput flipencoder;
        public void initialize(){
            flippy1 = hardwareMap.get(Servo.class, "flippy1");
            flippy2 = hardwareMap.get(Servo.class, "flippy2");
            flipencoder = hardwareMap.get(AnalogInput.class, "flipencoder");
        }
        public void setPosition(double pos){
            flippy1.setPosition(pos);
            flippy2.setPosition(pos);
        }
        public double getPosition(){
            return abs(1 - flipencoder.getVoltage() / 3.3) + .03;
        }
    }
    public void release(){
        //Waits after dropping the block on the bar and then goes to pick
        if(dropping == 2 && slidesPose < 10) {
            if(special_pick == 2){
                armtarget = 732;
                wristpose = .46;
                slidestarget = 0;
                flippose = .025;
                dropping = 1;
                special_pick = 1;
            }else {
                armtarget = 732;
                wristpose = .43;
                slidestarget = 0;
                flippose = .025;
                flipsafe = 2;
                dropping = 1;
            }
        }
    }
}
