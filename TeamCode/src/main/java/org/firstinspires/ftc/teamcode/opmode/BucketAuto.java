package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.concurrent.TimeUnit;


/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "BucketAuto", group = "AAA", preselectTeleOp = "Bucket_Tele")
public class BucketAuto extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private double forward = 1;

    private Follower follower;

    private Path forwards;
    private Path backwards;
    private Path push1;
    private Path push1ish;
    private Path push1ish2;
    private Path push2;
    private Path push2ish;
    private Path push3;
    private Path push3ish;
    private Path score1;
    private Path score2;
    private Path score2ish;
    private Path score3;
    private Path score3ish;
    private Path score4;
    private Path score4ish;
    private Path score5;
    private Path score5ish;
    private Path comeback1;
    private Path comeback1ish;
    private Path comeback1ish2;
    private Path comeback2;
    private Path comeback2ish;
    private Path comeback2ish2;
    private Path comeback3;
    private Path comeback3ish;
    private Path comeback3ish2;
    private PathChain MainCode;
    private PIDController controller;
    private PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int slidestarget = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;
    public double a = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime drivetime = new ElapsedTime();

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;
    public spin gripspinny;

    double mode = 1;
    double basketmove =1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1600 ;
    double armbasket = 2000;
    double twistbasket = .5;
    double wristbasket = .6;
    double slidespecimen = .5;
    double armspecimen = 1380 ;
    double wristspecimen = .3;
    double twistspecimen = .5;
    double armspecimenpickup = 60;
    double wristspecimenpickup = .51;
    double ticks = .002866;
    double xpress = 1;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    double just = 0;
    public RevTouchSensor limitfront;
    public RevTouchSensor limitfront2;
    public DigitalChannel limitwrist1;
    public double r1press = 1;
    public double armPose = 0;
    double slidesPose = 0;
    double wristpose = .5;
    double twistpose = .5;
    public double count = 1;
    public double flippose = 0;
    public flippy flip;
    public double flipsafe = 1;
    public double dropping = 1;
    public double special_pick = 1;
    public double first_score = 1;
    public double wrist_at = 0;
    public AnalogInput wristencoder;
    public double stopped = 1;



    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        flip = new flippy();
        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);
        score1 = new Path(new BezierCurve(new Point(7.2, 6.5, Point.CARTESIAN), new Point(11.1, 13, Point.CARTESIAN), new Point(5.8, 18.8, Point.CARTESIAN)));
        score1.setLinearHeadingInterpolation(0, Math.toRadians(317));
        push1 = new Path(new BezierCurve(new Point(3.3, 20.3, Point.CARTESIAN), new Point(6.3, 14.1, Point.CARTESIAN), new Point(13.2, 9.7, Point.CARTESIAN)));
        push1.setLinearHeadingInterpolation(Math.toRadians(317),0);
        push1ish = new Path(new BezierLine(new Point(13.2, 9.7, Point.CARTESIAN), new Point(20.2, 10.1, Point.CARTESIAN)));
        push1ish.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierLine(new Point(25.2, 10.1, Point.CARTESIAN), new Point(13.4, 10.9, Point.CARTESIAN)));
        score2.setLinearHeadingInterpolation(0, Math.toRadians(317));
        score2ish = new Path(new BezierCurve(new Point(13.4, 10.9, Point.CARTESIAN), new Point(7.8, 16.3, Point.CARTESIAN), new Point(6.3, 17.3, Point.CARTESIAN)));
        score2ish.setConstantHeadingInterpolation(Math.toRadians(317));
        push2 = new Path(new BezierLine(new Point(3.3, 20.3, Point.CARTESIAN), new Point(9.1, 19.7, Point.CARTESIAN)));
        push2.setLinearHeadingInterpolation(Math.toRadians(317),0);
        push2ish = new Path(new BezierCurve(new Point(9.1, 19.7, Point.CARTESIAN), new Point(15.3, 19.6, Point.CARTESIAN), new Point(16.3, 19.5, Point.CARTESIAN), new Point(20.2, 19.6, Point.CARTESIAN)));
        push2ish.setConstantHeadingInterpolation(0);
        score3 = new Path(new BezierLine(new Point(25.2, 19.6, Point.CARTESIAN), new Point(12.7, 15.5, Point.CARTESIAN)));
        score3.setLinearHeadingInterpolation(0, Math.toRadians(317));
        score3ish = new Path(new BezierCurve(new Point(12.7, 11.5,Point.CARTESIAN), new Point(8.2, 15.5, Point.CARTESIAN), new Point(6.4, 17.4, Point.CARTESIAN)));
        score3ish.setConstantHeadingInterpolation(Math.toRadians(317));
        push3 = new Path(new BezierCurve(new Point(13, 11, Point.CARTESIAN), new Point(22.7, 7.5, Point.CARTESIAN), new Point(34.7, 9.7, Point.CARTESIAN)));
        push3.setLinearHeadingInterpolation(Math.toRadians(317),Math.toRadians(90));
        push3ish = new Path(new BezierCurve(new Point(34.7, 9.7, Point.CARTESIAN), new Point(35, 13.2, Point.CARTESIAN), new Point(35.2, 13.7, Point.CARTESIAN)));
        push3ish.setConstantHeadingInterpolation(Math.toRadians(90));
        score4 = new Path(new BezierLine(new Point(35.2, 17.7, Point.CARTESIAN), new Point(19.3, 10.5, Point.CARTESIAN)));
        score4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(317));
        score4ish = new Path(new BezierCurve(new Point(16.3, 7.5, Point.CARTESIAN), new Point(10, 14.3, Point.CARTESIAN), new Point(6.7, 17.4, Point.CARTESIAN)));
        score4ish.setConstantHeadingInterpolation(Math.toRadians(317));
        comeback1 = new Path(new BezierCurve(new Point(15.3, 13.3, Point.CARTESIAN), new Point(33.4, 10.4, Point.CARTESIAN), new Point(39.7, 10.2, Point.CARTESIAN)));
        comeback1.setLinearHeadingInterpolation(Math.toRadians(317), 0);
        comeback1ish = new Path(new BezierCurve(new Point(39.7, 10.2, Point.CARTESIAN), new Point(51, 0, Point.CARTESIAN), new Point(51.7, -9.4, Point.CARTESIAN), new Point(51.7, -13.4, Point.CARTESIAN)));
        comeback1ish.setLinearHeadingInterpolation(0, Math.toRadians(90));
        flip.initialize();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
        initializations();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void init_loop(){
        arm();
    }
    @Override
    public void loop() {
        follower.update();
        if(a == 0){
            a = 1;
        }
        arm();
        if(a == 2 && drivetime.time(TimeUnit.MILLISECONDS) > 400){
            if(abs(armPose - armtarget) < 100) {
                slidestarget = 1608;
            }
            if(abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100){
                follower.followPath(score1);
                flippose = .5;
                a = 3;
            }

        }
        if(a == 3) {
            if (follower.atParametricEnd() || !follower.isBusy()) {
                telemetryA.addData("going forward", forward);
                follower.telemetryDebug(telemetryA);
                stopped = 2;
            }
            if(stopped == 2){
                if(forward < 7) {
                    if (forward == 1) {
                        drivetime.reset();
                        forward = 1.1;
                    } else if (drivetime.time(TimeUnit.MILLISECONDS) > 600 && forward == 1.1) {
                        gripspinny.setPower(.2);
                        wristpose = .6;
                        drivetime.reset();
                        forward = 1.5;
                    } else if (forward == 1.5 && drivetime.time(TimeUnit.MILLISECONDS) > 400) {
                        wristpose = .293;
                        flippose = .613;
                        if(abs(wristpose - wrist_at) < .05) {
                            armtarget = 0;
                            slidestarget = (int) (6 * slideticks * 2);
                            twistpose = 0;

                        }
                        gripspinny.setPower(-1);
                        if (abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100 && abs(wristpose - wrist_at) < .05){
                            follower.followPath(push1);
                            stopped = 1;
                            forward = 2;
                        }
                        gripspinny.setPower(-1);

                    } else if (forward == 2) {
                        follower.followPath(push1ish);
                        stopped = 1;
                        forward = 2.5;
                    }else if(forward == 2.5){
                        drivetime.reset();
                        forward = 3;
                    }else if (forward == 3) {
                        if(!follower.isBusy()) {
                            if (limitwrist1.getState() && drivetime.time(TimeUnit.MILLISECONDS) < 1500) {
                                flippose = .692;
                                armtarget = 0;
                                wristpose = .293;
                            } else {
                                armtarget = (int) 1742;
                                flippose = .5;
                                wristpose = .55;
                                twistpose = 0;
                                if (abs(armPose - armtarget) < 100) {
                                    slidestarget = 1608;
                                }
                                if (abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100) {
                                    follower.followPath(score2);
                                    stopped = 1;
                                    forward = 4;
                                }
                            }
                        }

                    } else if (forward == 4) {
                        follower.followPath(score2ish);
                        stopped = 1;
                        forward = 5;
                    } else if (forward == 5) {
                        wristpose = .6;
                        drivetime.reset();
                        forward = 5.25;
                    } else if(forward == 5.25 && drivetime.time(TimeUnit.MILLISECONDS) > 1000){
                        gripspinny.setPower(1);
                        drivetime.reset();
                        forward = 5.5;
                    }else if(forward == 5.5 && drivetime.time(TimeUnit.MILLISECONDS) > 250){
                        wristpose = .293;
                        flippose = .613;
                        if(abs(wristpose - wrist_at) < .05) {
                            armtarget = 0;
                            slidestarget = (int) (5 * slideticks * 2);
                            twistpose = 0;

                        }
                        gripspinny.setPower(-1);
                        if(abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100 && abs(wristpose - wrist_at) < .05) {
                            follower.followPath(push2);
                            stopped = 1;
                            forward = 6;
                        }
                    }else if (forward == 6) {
                        follower.followPath(push2ish);
                        stopped = 1;
                        forward = 6.5;
                    }
                    if(forward == 6.5){
                        drivetime.reset();
                        forward = 7;
                    }
                }else{
                    if (forward == 7) {
                        if(!follower.isBusy()) {
                            if (limitwrist1.getState() && drivetime.time(TimeUnit.MILLISECONDS) < 1500) {
                                flippose = .692;
                                armtarget = 0;
                                wristpose = .293;
                            } else {
                                armtarget = (int) 1742;
                                flippose = .5;
                                wristpose = .55;
                                twistpose = 0;
                                if (abs(armPose - armtarget) < 100) {
                                    slidestarget = 1608;
                                }
                                if (abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100) {
                                    follower.followPath(score3);
                                    stopped = 1;
                                    forward = 8;
                                }
                            }
                        }
                    } else if (forward == 8) {
                        follower.followPath(score3ish);
                        stopped = 1;
                        forward = 9;
                    }else if (forward == 9) {
                        gripspinny.setPower(1);
                        wristpose = .6;
                        drivetime.reset();
                        forward = 9.5;
                    } else if(forward == 9.5 && drivetime.time(TimeUnit.MILLISECONDS) > 1000){
                        wristpose = .293;
                        flippose = .613;
                        if(abs(wristpose - wrist_at) < .05) {
                            armtarget = 0;
                            slidestarget = (int) (4 * slideticks * 2);
                            twistpose = .28;

                        }
                        gripspinny.setPower(-1);
                        if(abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100 && abs(wristpose - wrist_at) < .05) {
                            follower.followPath(push3);
                            stopped = 1;
                            forward = 10;
                        }
                    } else if (forward == 10) {
                        follower.followPath(push3ish);
                        stopped = 1;
                        forward = 10.5;
                    }else if(forward == 10.5){
                        drivetime.reset();
                        forward = 11;
                    } else if (forward == 11) {
                        if(limitwrist1.getState() && drivetime.time(TimeUnit.MILLISECONDS) < 1500){
                            flippose = .692;
                            armtarget = 0;
                            wristpose = .293;
                        }
                        else {
                            armtarget = (int) 1742;
                            flippose = .5;
                            wristpose = .55;
                            twistpose = 0;
                            if (abs(armPose - armtarget) < 100) {
                                slidestarget = 1608;
                            }
                            if (abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100) {
                                follower.followPath(score4);
                                stopped = 1;
                                forward = 12;
                            }
                        }
                    } else if (forward == 12) {
                        armtarget = (int) 1742;
                        flippose = .5;
                        wristpose = .55;
                        if(abs(armPose - armtarget) < 100) {
                            slidestarget = 1608;
                        }
                        twistpose = 0;
                        if(abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100) {
                            follower.followPath(score4ish);
                            stopped = 1;
                            forward = 13;
                        }
                    } else if (forward == 13) {
                        drivetime.reset();
                        forward = 13.25;
                    } else if(forward == 13.25 && drivetime.time(TimeUnit.MILLISECONDS) > 800){
                        gripspinny.setPower(.5);
                        wristpose = .6;
                        drivetime.reset();
                        forward = 13.5;
                    }else if(forward == 13.5 && drivetime.time(TimeUnit.MILLISECONDS) > 300){
                        wristpose = .293;
                        flippose = .613;
                        if(abs(wristpose - wrist_at) < .05) {
                            armtarget = 1890;
                            slidestarget = 0;
                            twistpose = 0;
                        }
                        if(abs(armPose - armtarget) < 100 && abs(slidesPose - slidestarget) < 100 && abs(wristpose - wrist_at) < .05) {
                            follower.followPath(comeback1);
                            armtarget = 1890;
                            wristpose = .34;
                            twistpose = 0;
                            slidestarget = 20;
                            flippose = .583;
                            stopped = 1;
                            forward = 14;
                        }
                        gripspinny.setPower(0);

                    } else if (forward == 14) {
                        follower.followPath(comeback1ish);
                        stopped = 1;
                        forward = 15;
                    }
                }
            }
        }
    }
    public void initializations(){
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        wristencoder = hardwareMap.get(AnalogInput.class, "wristencoder");
        gripspinny = new spin();
        gripspinny.initialize();
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        limitwrist1 = hardwareMap.get(DigitalChannel.class, "limitwrist1");
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        limitfront2 = hardwareMap.get(RevTouchSensor.class, "limitfront2");
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armtarget = 0;
        slidestarget = 0;
        wristpose = .8;
        flippose = .235;
        twistpose = 0;
    }
    public void arm(){
        toplimit = 1406;
        wrist_at = abs(1 - wristencoder.getVoltage() / 3.3);
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = -slides.getCurrentPosition()/slideticks * .03 / 19.6;
        armf = .001 + -slides.getCurrentPosition()/slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        slides.setPower(-power);
        armcontroller.setPID(armp, armi, armd);
        armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        Arm1.setPower(-armpower);
        Arm2.setPower(-armpower);
        if(a == 1){
            a = 2;
            armtarget = 732;
            slidestarget = 648;
            wristpose = .69;
            twistpose = 0;
            flippose = .651;
            gripspinny.setPower(-1);
        }
        flip.setPosition(flippose);
        wristy.setPosition(wristpose - .04);
        twisty.setPosition(twistpose + .028);

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
    public void safeflip(){
        if(flipsafe == 2 && flip.getPosition() < .45){
            twistpose = .56;
            flipsafe = 1;
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
