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
@Autonomous (name = "GreenAuto", group = "AAA", preselectTeleOp = "Codethatworks"
)
public class GreenAuto extends OpMode {
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
    private Path score6;
    private Path score6ish;
    private Path score6ish2;
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
    public AnalogInput wristencoder;
    public double wrist_at = 0;
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
    public double stick = 1;
    public double missed = 0;



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
        score1 = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(27, 16, Point.CARTESIAN)));
        score1.setConstantHeadingInterpolation(0);
        push1 = new Path(new BezierCurve(new Point(15, -12.1, Point.CARTESIAN),new Point(19, -18.1, Point.CARTESIAN), new Point(22.8, -21, Point.CARTESIAN), new Point(48.2, -18, Point.CARTESIAN),new Point(48.8, -30, Point.CARTESIAN)));
        push1ish = new Path(new BezierCurve(new Point(48.8, -30, Point.CARTESIAN), new Point(45, -30, Point.CARTESIAN), new Point(38.2, -30, Point.CARTESIAN), new Point(13.9, -30, Point.CARTESIAN)));
        push1.setConstantHeadingInterpolation(0);
        push1ish.setConstantHeadingInterpolation(0);
        push2 = new Path(new BezierCurve(new Point(14.9, -30, Point.CARTESIAN),new Point(27.2, -30, Point.CARTESIAN), new Point(39.5, -30, Point.CARTESIAN), new Point(44.7, -32, Point.CARTESIAN),new Point(47.5, -39.6, Point.CARTESIAN)));
        push2ish = new Path(new BezierCurve(new Point(47.5, -39.6, Point.CARTESIAN), new Point(44.7, -39.6, Point.CARTESIAN), new Point(30.5, -39.7, Point.CARTESIAN), new Point(14.8, -40, Point.CARTESIAN)));
        push2.setConstantHeadingInterpolation(0);
        push2ish.setConstantHeadingInterpolation(0);
        push3 = new Path(new BezierCurve(new Point(13.8, -40, Point.CARTESIAN), new Point(27.3, -39.9, Point.CARTESIAN), new Point(42, -40.7, Point.CARTESIAN), new Point(48, -41.3, Point.CARTESIAN),new Point(50.6, -46.8, Point.CARTESIAN)));
        push3ish = new Path(new BezierCurve(new Point(50.6, -46.8, Point.CARTESIAN), new Point(39.2, -46.8, Point.CARTESIAN), new Point(16.7, -46.8, Point.CARTESIAN), new Point(3.5, -46.8, Point.CARTESIAN)));
        push3.setConstantHeadingInterpolation(0);
        push3ish.setConstantHeadingInterpolation(0);
        score2 = new Path(new BezierCurve(new Point(4.7, -34, Point.CARTESIAN), new Point(9, -16.2, Point.CARTESIAN), new Point(13.3, -1.4, Point.CARTESIAN), new Point(20.6, 13.1, Point.CARTESIAN), new Point(28, 15.4, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(0);
        score2ish = new Path(new BezierLine(new Point(29, 11.4, Point.CARTESIAN), new Point(31, 17.2, Point.CARTESIAN)));
        score2ish.setConstantHeadingInterpolation(0);
        comeback1 = new Path(new BezierCurve(new Point(28.8, 19, Point.CARTESIAN), new Point(22.2, 2.8, Point.CARTESIAN), new Point(16.9, -12.3, Point.CARTESIAN), new Point(10, -16.8, Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(0);
        comeback1ish = new Path(new BezierLine(new Point(10, -16.8, Point.CARTESIAN), new Point(2.4, -16.9, Point.CARTESIAN)));
        comeback1ish.setConstantHeadingInterpolation(0);
        score3 = new Path(new BezierCurve(new Point(6.1, -14.9, Point.CARTESIAN), new Point(12.1, .7, Point.CARTESIAN), new Point(17.3, 11.4, Point.CARTESIAN), new Point(27, 11.1, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(0);
        score4 = new Path(new BezierCurve(new Point(6.1, -14.9, Point.CARTESIAN), new Point(12.1, .7, Point.CARTESIAN), new Point(17.3, 11.4, Point.CARTESIAN), new Point(28, 11.6, Point.CARTESIAN)));
        score4.setConstantHeadingInterpolation(0);
        score5 = new Path(new BezierCurve(new Point(6.1, -14.9, Point.CARTESIAN), new Point(12.1, .7, Point.CARTESIAN), new Point(17.3, 11.4, Point.CARTESIAN), new Point(27, 12.6, Point.CARTESIAN)));
        score5.setConstantHeadingInterpolation(0);
        score3ish = new Path(new BezierLine(new Point(28, 11.6, Point.CARTESIAN), new Point(31, 15.4, Point.CARTESIAN)));
        score3ish.setConstantHeadingInterpolation(0);
        score6 = new Path(new BezierCurve(new Point(4.1, -14.9, Point.CARTESIAN), new Point(10.6, -12.4, Point.CARTESIAN), new Point(13, -5, Point.CARTESIAN), new Point(13, 0, Point.CARTESIAN), new Point(13, 10, Point.CARTESIAN), new Point(13, 15.2, Point.CARTESIAN)));
        score6ish = new Path(new BezierCurve(new Point(13, 30.7, Point.CARTESIAN), new Point(13, 40.9, Point.CARTESIAN),new Point(13, 50.7, Point.CARTESIAN)));
        score6ish2 = new Path(new BezierCurve(new Point(13, 57.9, Point.CARTESIAN), new Point(12, 64, Point.CARTESIAN), new Point(4.9, 71.4, Point.CARTESIAN)));
        MainCode = new PathChain(score6, score6ish);
        follower.followPath(score1);
        flip.initialize();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
        initializations();
    }
    @Override
    public void init_loop(){
        arm();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        if(a == 0){
            a = 1;
        }
        if(first_score == 1){
            if(limitfront.isPressed() || limitfront2.isPressed()){
                gripspinny.setPower(1);
                wristpose = .5;
                slidestarget = 0;
                dropping = 2;
                first_score = 2;
            }
        }
        arm();
        safeflip();
        release();
        getwall();
        follower.update();
        if(follower.atParametricEnd() || !follower.isBusy()) {
            if(count <= 1) {
                if (forward == 1) {
                    //finish first score and start the movements to push the first block
                    drivetime = new ElapsedTime();
                    follower.followPath(push1);
                    forward = 1.5;
                } else if (forward == 1.5) {
                    //finish pushing the first block
                    follower.followPath(push1ish);
                    wristpose = .5;
                    slidestarget = 0;
                    dropping = 2;
                    gripspinny.setPower(0);
                    forward = 2;
                } else if (forward == 2) {
                    //start pushing block 2
                    follower.followPath(push2);
                    forward = 2.5;
                } else if (forward == 2.5) {
                    //finish pushing block 2
                    follower.followPath(push2ish);
                    forward = 3;
                } else if (forward == 3) {
                    //start push block 3
                    follower.followPath(push3);
                    gripspinny.setPower(-1);
                    forward = 3.5;
                } else if (forward == 3.5) {
                    //finish pushing block 3 and pick block from wall

                    follower.followPath(push3ish);
                    forward = 4;
                } else if (forward == 4) {
                    //go to score the second time
                    armtarget = 732;
                    slidestarget = 648;
                    wristpose = .25;
                    twistpose = 0;
                    flippose = .651;
                    stick = 2;
                    follower.followPath(score2);
                    //forward = 4.5;
                    count++;
                    forward = 5;
                }
            } else if(count <= 5) {
                //loop for scoring all other blocks
                if (forward == 5) {
                    //comeback to human player to pick up a block
                    gripspinny.setPower(1);
                    wristpose = .5;
                    slidestarget = 0;
                    if (count == 5) {
                        //used for the last pick to pick the yellow from the wall
                        special_pick = 2;
                    }
                    dropping = 2;
                    follower.followPath(comeback1);
                    forward = 5.5;
                } else if (forward == 5.5) {
                    gripspinny.setPower(-1);
                    //pick block from human player
                    follower.followPath(comeback1ish);
                    forward = 6;
                    if (count == 5) {
                        //park for now, score in bucket in the future
                        forward = 10;
                        count = 6;
                    }
                } else if (forward == 6) {
                    //go to score the block
                    armtarget = 732;
                    slidestarget = 648;
                    wristpose = .25;
                    twistpose = 0;
                    flippose = .651;
                    stick = 2;
                    count += 1;
                    if(count == 5){
                        follower.followPath(score4);
                        forward = 5;
                    }else if(count == 4){
                        follower.followPath(score4);
                        forward = 6.5;
                    }else {
                        forward = 5;
                        follower.followPath(score3);
                    }

                } else if (forward == 6.5) {
                    //scoot over after scoring

                    follower.followPath(score3ish);
                    forward = 5;

                }
            }else{
                if(forward == 10){
                    //hold final pose for now
                    slidestarget = 0;
                    wristpose = .5;
                    twistpose = 0;
                    flippose = .635;
                    if(missed == 1){
                        forward = 20;
                    }else {
                        follower.followPath(score6);
                        forward = 10.5;
                    }
                }else if(forward == 10.5){
                    follower.followPath(score6ish);
                    armtarget = 1619;
                    slidestarget = 330;
                    wristpose = .39;
                    twistpose = 0;
                    flippose = .522;
                    forward = 10.75;
                }else if(forward == 10.75){
                    drivetime.reset();
                    forward = 11;
                }else if(forward == 11 && drivetime.time(TimeUnit.MILLISECONDS) > 100){
                    armtarget = 1635;
                    slidestarget = 1634;
                    wristpose = .4;
                    twistpose = 0;
                    flippose = .53;
                    follower.followPath(score6ish2);
                    forward = 12;
                }else if(forward == 12){
                    drivetime.reset();
                    forward = 13;
                }
            }
            if(forward == 13){
                if(drivetime.time(TimeUnit.MILLISECONDS) > 300){
                    gripspinny.setPower(.4);
                    drivetime.reset();
                    forward = 14;
                }
            }else if (forward == 14){
                if(drivetime.time(TimeUnit.MILLISECONDS) > 200){
                    armtarget = 1619;
                    slidestarget = 0;
                    wristpose = .5;
                    twistpose = 0;
                    flippose = .3;
                }
            }
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
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
        if(-250 < slidesPose - slidestarget && slidesPose - slidestarget < 250 && (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1)){
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (otherPose < bottomlimit && gamepad2.right_stick_y > 0){
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }else if(otherPose > toplimit && gamepad2.right_stick_y < 0){
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }else{
                slides.setPower(gamepad2.right_stick_y/2);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }
        }else {
            slides.setPower(-power);
        }
        armcontroller.setPID(armp, armi, armd);
        armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if(-200 < armPose - armtarget && armPose - armtarget < 200 && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)){
            Arm1.setPower(gamepad2.left_stick_y/2);
            Arm2.setPower(gamepad2.left_stick_y/2);
            armtarget = (int) (armPose);
        }else {
            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
        }
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
        twisty.setPosition(twistpose + .048);

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
    public void getwall(){
        if(stick == 2 && abs(wristpose - wrist_at) < .05){
            wristpose = .69;
            stick = 1;
        }
    }
}
