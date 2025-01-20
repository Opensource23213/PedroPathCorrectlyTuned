package org.firstinspires.ftc.teamcode.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="ArmTest", group="ABC Opmode")
//@Disabled
public class ArmTest extends OpMode {
    private PIDController controller;
    private PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int target = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;
    public AnalogInput wristencoder;
    public double wrist_at;

    double mode = 2;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;
    double bottomlimit = .25;
    double slidebasket = 18.77;
    double armbasket = 90;
    double twistbasket = .5;
    double wristbasket = .86;
    double slidespecimen = 0;
    double armspecimen = 61 * armticks;
    double wristspecimen = .28;
    double twistspecimen = .5;
    double armspecimenpickup = -1.8;
    double wristspecimenpickup = .63;
    double xpress = 1;
    public flippy flip;
    public spin gripspinny;
    double wristpose = .5;
    double twistpose = 0;
    double flippose = 0;
    public double slidestarget = 0;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flip = new flippy();
        gripspinny = new spin();
        flip.initialize();
        gripspinny.initialize();
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        wristencoder = hardwareMap.get(AnalogInput.class, "wristencoder");
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flippose = .55;
        wristpose = .5;
        twistpose = 0;
    }

    @Override
    public void loop() {
        arm();
        if(gamepad2.a){
            gripspinny.setPower(1);
        }else if(gamepad2.b){
            gripspinny.setPower(-1);
        }  else{
            gripspinny.setPower(0);
        }
        if(gamepad1.a){
            //pick
            armtarget = 0;
            slidestarget = (int) (2 * slideticks * 2);
            wristpose = .293;
            twistpose = 0;
            flippose = .613;
        }
        else if(gamepad1.b){
            //0
            flippose = .55;
            wristpose = .5;
            twistpose = 0;
        }
        if (wristpose <= 1 && gamepad2.dpad_up){
            wristpose += .001;
        }
        if (twistpose <= 1 && gamepad2.dpad_right){
            twistpose += .001;
        }
        if (wristpose >= 0 && gamepad2.dpad_down){
            wristpose -= .001;
        }
        if (twistpose >= 0 && gamepad2.dpad_left){
            twistpose -= .001;
        }
        if (flippose >= 0 && gamepad1.dpad_down){
            flippose -= .001;
        }
        if (flippose <= 1 && gamepad1.dpad_up){
            flippose += .001;
        }
    }

    public void arm(){
        wrist_at = abs(1 - wristencoder.getVoltage() / 3.3) + .03;
        toplimit = 1506 + (2 * slideticks * 2);
        controller.setPID(p, i, d);
        double newpos = -312;
        double slidesPose = -slides.getCurrentPosition() * 2;
        armd = -slides.getCurrentPosition() / slideticks * .03 / 19.6;
        armf = .001 + -slides.getCurrentPosition() / slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        if (-250 < slidesPose - slidestarget && slidesPose - slidestarget < 250 && (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1)) {
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (otherPose < bottomlimit && gamepad2.right_stick_y > 0) {
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
        double ticks = .002866;
        double armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
         if (abs(abs(armPose) - abs(armtarget)) < 150 && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)) {
            if (armPose < newpos + 45 && gamepad2.left_stick_y > 0) {
                Arm1.setPower(0);
                Arm2.setPower(0);
                armtarget = (int) (armPose);
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
        telemetry.addData("Flippy position", flip.getPosition());
        telemetry.addData("Wrist position", wrist_at);
        telemetry.addData("Twist position", twisty.getPosition());
        telemetry.update();
        wristy.setPosition(wristpose - .04);
        twisty.setPosition(twistpose + .019);
        flip.setPosition(flippose);
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
            return abs(1 - flipencoder.getVoltage() / 3.3);
        }
    }
}



