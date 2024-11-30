package org.firstinspires.ftc.teamcode.opmode;

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
@Autonomous (name = "Score5Auto", group = "AAA")
public class Score5Auto extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private double forward = 0 ;

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
    public double count = 0;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */ do you want to
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);
        score1 = new Path(new BezierCurve(new Point(3.4, 3.7, Point.CARTESIAN), new Point(10.8, 7.3, Point.CARTESIAN), new Point(20.8, 12.1, Point.CARTESIAN), new Point(28, 12.9, Point.CARTESIAN)));
        score1.setConstantHeadingInterpolation(0);
        push1 = new Path(new BezierCurve(new Point(31, 12.9, Point.CARTESIAN), new Point(25.3, 10, Point.CARTESIAN), new Point(22.8, 1.8, Point.CARTESIAN), new Point(23.2, -12.7, Point.CARTESIAN)));
        push1.setConstantHeadingInterpolation(0);
        push1ish = new Path(new BezierCurve(new Point(23.2, -12.7, Point.CARTESIAN), new Point(27.3, -19.5, Point.CARTESIAN), new Point(30.9, -22.6, Point.CARTESIAN), new Point(47.1, -24.2, Point.CARTESIAN)));
        push1ish.setConstantHeadingInterpolation(0);
        push1ish2 = new Path(new BezierCurve(new Point(47.1, -24.2, Point.CARTESIAN), new Point(49.6, -31.3, Point.CARTESIAN), new Point(37.8, -31.3, Point.CARTESIAN), new Point(13, -31, Point.CARTESIAN)));
        push1ish2.setConstantHeadingInterpolation(0);
        push2 = new Path(new BezierCurve(new Point(13, -31, Point.CARTESIAN), new Point(20, -31, Point.CARTESIAN), new Point(30, -31, Point.CARTESIAN), new Point(43.9, -32.7, Point.CARTESIAN), new Point(48.3, -34.6, Point.CARTESIAN), new Point(48.5, -39.3, Point.CARTESIAN)));
        push2ish = new Path(new BezierCurve(new Point(48.5, -39.3, Point.CARTESIAN), new Point(34, -40.2, Point.CARTESIAN), new Point(12.8, -38.9, Point.CARTESIAN)));
        push2.setConstantHeadingInterpolation(0);
        push2ish.setConstantHeadingInterpolation(0);
        push3 = new Path(new BezierCurve(new Point(23.6, -37.5, Point.CARTESIAN), new Point(45.7, -38, Point.CARTESIAN), new Point(47.4, -42.2, Point.CARTESIAN)));
        push3.setLinearHeadingInterpolation(0,Math.toRadians(180));
        push3ish = new Path(new BezierCurve(new Point(49.4, -47.9, Point.CARTESIAN), new Point(46.3, -47.9, Point.CARTESIAN), new Point(20.4, -43.7, Point.CARTESIAN), new Point(5.4, -43.7, Point.CARTESIAN)));
        push3ish.setConstantHeadingInterpolation(Math.toRadians(180));
        score2 = new Path(new BezierLine(new Point(4.4, -43.7, Point.CARTESIAN), new Point(8.6, -26.2, Point.CARTESIAN)));
        score2ish = new Path(new BezierCurve(new Point(13, -8.2, Point.CARTESIAN), new Point(19.8, 6.6, Point.CARTESIAN), new Point(29, 16.6, Point.CARTESIAN)));
        score2.setConstantHeadingInterpolation(Math.toRadians(0));
        score2ish.setConstantHeadingInterpolation(Math.toRadians(0));
        comeback1 = new Path(new BezierLine(new Point(30.7, 16.6, Point.CARTESIAN), new Point(22.1, 16.2, Point.CARTESIAN)));
        comeback1ish = new Path(new BezierCurve(new Point(16.3, 9, Point.CARTESIAN), new Point(16, -3.7, Point.CARTESIAN), new Point(16.3 , -15.8, Point.CARTESIAN), new Point(5.1, -17.5, Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(Math.toRadians(180));
        comeback1ish.setConstantHeadingInterpolation(Math.toRadians(180));
        score3 = new Path(new BezierLine(new Point(5.1, -17.5, Point.CARTESIAN), new Point(10.5, -17.4, Point.CARTESIAN)));
        score3ish = new Path(new BezierCurve(new Point(13.2, -8, Point.CARTESIAN), new Point(14.5, 10.1, Point.CARTESIAN), new Point(18.4, 16.9, Point.CARTESIAN), new Point(28, 18.2, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(Math.toRadians(0));
        score3ish.setConstantHeadingInterpolation(Math.toRadians(0));
        score4 = new Path(new BezierLine(new Point(9.9, -19.2 , Point.CARTESIAN), new Point(10.5, -12.6, Point.CARTESIAN)));
        score4ish = new Path(new BezierCurve(new Point(13.7, -2, Point.CARTESIAN), new Point(19.3, 9.5, Point.CARTESIAN), new Point(24.7, 10.7, Point.CARTESIAN), new Point(27.5, 17, Point.CARTESIAN)));
        score4.setConstantHeadingInterpolation(Math.toRadians(5));
        score4ish.setConstantHeadingInterpolation(Math.toRadians(5));
        score5 = new Path(new BezierLine(new Point(9.9, -19.2 , Point.CARTESIAN), new Point(10.5, -12.6, Point.CARTESIAN)));
        score5ish = new Path(new BezierCurve(new Point(13.7, -2, Point.CARTESIAN), new Point(19.3, 9.5, Point.CARTESIAN), new Point(24.7, 10.7, Point.CARTESIAN), new Point(27.5, 17, Point.CARTESIAN)));
        score5.setConstantHeadingInterpolation(Math.toRadians(0));
        score5ish.setConstantHeadingInterpolation(Math.toRadians(0));
        comeback2 = new Path(new BezierCurve(new Point(22, 17, Point.CARTESIAN), new Point(21, 7.2, Point.CARTESIAN), new Point(20, -1.4, Point.CARTESIAN)));
        comeback2ish = new Path(new BezierCurve(new Point(20, -1.4, Point.CARTESIAN), new Point(20, -6.9, Point.CARTESIAN),new Point(20, -11.9, Point.CARTESIAN), new Point(20, -14, Point.CARTESIAN), new Point(13, -16, Point.CARTESIAN)));
        comeback2.setConstantHeadingInterpolation(Math.toRadians(190));
        comeback2ish.setConstantHeadingInterpolation(Math.toRadians(190));
        comeback3 = new Path(new BezierCurve(new Point(22, 17, Point.CARTESIAN), new Point(21, 7.2, Point.CARTESIAN), new Point(20, -1.4, Point.CARTESIAN)));
        comeback3ish = new Path(new BezierCurve(new Point(20, -1.4, Point.CARTESIAN), new Point(20, -6.9, Point.CARTESIAN),new Point(20, -11.9, Point.CARTESIAN), new Point(20, -15, Point.CARTESIAN), new Point(13, -17, Point.CARTESIAN)));
        comeback3.setConstantHeadingInterpolation(Math.toRadians(195));
        comeback3ish.setConstantHeadingInterpolation(Math.toRadians(195));
        MainCode = new PathChain(score1, push1, push1ish, push2, push2ish, push3, push3ish, score2, comeback1, comeback1ish, score3, score3ish, score3, score3ish);
        follower.followPath(score1);

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
        a = 1;
        arm();
        if(forward == 0){
            armtarget = (int) (640);
            slidestarget = (int) (5 * slideticks * 2);
            wristpose = .75;
            twistpose = .5;
            forward = 1;
            count += 1;
        }
        follower.update();
        if (follower.atParametricEnd()) {
            if (forward == 1) {
                forward = 2;
                follower.followPath(push1);
                slidestarget = 0;
                gripspinny.setPower(1);
                wristpose = .47;
            } else if(forward == 2){
                forward = 2.5;
                follower.followPath(push1ish);
                armtarget = 0;
                wristpose = 0;
                twistpose = .5;
            }else if(forward == 2.5){
                forward = 3;
                follower.followPath(push1ish2);
            }else if(forward == 3){
                forward = 4;
                follower.followPath(push2);
            }else if(forward == 4) {
                forward = 5;
                follower.followPath(push2ish);
            }else if(forward == 5){
                forward = 6;
                follower.followPath(push3);
            }else if(forward == 6) {
                forward = 7;
                follower.followPath(push3ish);
                armtarget = (int) armspecimenpickup;
                wristpose = wristspecimenpickup;
                twistpose = .5;
                gripspinny.setPower(-1);
            }
            else if(forward == 7) {
                forward = 7.5;
                follower.followPath(score2);
                wristpose = .8;
            }else if(forward == 7.5) {
                forward = 8;
                follower.followPath(score2ish);
                armtarget = (int) (640);
                slidestarget = (int) (5 * slideticks * 2);
                wristpose = .75 ;
                twistpose = .5;
                gripspinny.setPower(0);
                count += 1;
            }else if(forward == 8) {
                forward = 9;
                /*if(count == 3){
                    follower.followPath(comeback2);
                }else if(count == 4){
                    follower.followPath(comeback3);
                }else {
                    follower.followPath(comeback1);
                }*/
                follower.followPath(comeback1);
                slidestarget = 0;
                gripspinny.setPower(1);
                wristpose = .47;
            }
            else if(forward == 9) {
                forward = 10;
                /*if(count == 3){
                    follower.followPath(comeback2ish);
                }else if(count == 4){
                    follower.followPath(comeback3ish);
                }else {
                    follower.followPath(comeback1ish);
                }*/
                follower.followPath(comeback1ish);
                armtarget = (int) armspecimenpickup;
                wristpose = wristspecimenpickup;
                slidestarget = 0;
                twistpose = .5;
                gripspinny.setPower(-1);
            }else if(forward == 10) {
                forward = 11;
                if(count > 82){
                    follower.followPath(score4);
                }else {
                    follower.followPath(score3);
                }
                wristpose = .8;
            }
            else if(forward == 11) {
                forward = 8;
                count += 1;
                if(count == 82){
                    follower.followPath(score4ish);
                }else if(count == 83){
                    follower.followPath(score5ish);
                }else {
                    follower.followPath(score3ish);
                }
                armtarget = (int) (660);
                slidestarget = (int) (5.3 * slideticks * 2);
                wristpose = .75 ;
                twistpose = .5;
                gripspinny.setPower(0);
                if(count >= 5){
                    forward = 12;
                }
            }
            else{

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
        gripspinny = new spin();
        gripspinny.initialize();
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        imu = hardwareMap.get(IMU.class, "imu");
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
        armtarget = (int) (14.5 * armticks);
        slidestarget = 0;
    }
    public void arm(){
        toplimit = 1406;
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
        if(a == 1) {
            wristy.setPosition(wristpose);
            twisty.setPosition(twistpose);
        }

    }
    public class spin{
        public CRServo spinny1;
        public CRServo spinny2;
        public void initialize(){
            spinny1 = hardwareMap.get(CRServo.class, "spinny1");
            spinny2 = hardwareMap.get(CRServo.class, "spinny2");
            spinny1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void setPower(double power){
            spinny1.setPower(power);
            spinny2.setPower(power);
        }
        public double getPower(){
            return spinny1.getPower();
        }
    }
}
