package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
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
@Disabled
@Autonomous (name = "PedroAuto", group = "Autonomous Pathing Tuning")
public class PedroAuto extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private double forward = 1;

    private Follower follower;
    private Path push1;
    private Path push1ish;
    private Path push2;
    private Path push2ish;
    private Path push3;
    private Path push3ish;
    private Path score1;
    private Path score2;
    private Path score3;
    private Path score3ish;
    private Path score5;
    private Path comeback1;
    private Path comeback1ish;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        score1 = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(28, 14, Point.CARTESIAN)));
        score1.setConstantHeadingInterpolation(0);
        push1 = new Path(new BezierCurve(new Point(28, 14, Point.CARTESIAN), new Point(27.4, 11.4, Point.CARTESIAN), new Point(23.8, 5.2, Point.CARTESIAN), new Point(22, 1.1, Point.CARTESIAN), new Point(21.8, -7.4, Point.CARTESIAN), new Point(22.9, -16, Point.CARTESIAN)));
        push1.setConstantHeadingInterpolation(0);
        push1ish = new Path(new BezierCurve(new Point(22.9, -16, Point.CARTESIAN), new Point(28.2, -23, Point.CARTESIAN),new Point(33.2, -23, Point.CARTESIAN),new Point(39, -23, Point.CARTESIAN), new Point(42.1, -23, Point.CARTESIAN), new Point(46.9, -23.9, Point.CARTESIAN), new Point(52, -25, Point.CARTESIAN), new Point(52, -33.5, Point.CARTESIAN), new Point(42.6, -33.3, Point.CARTESIAN), new Point(32, -33.3, Point.CARTESIAN), new Point(25, -33.3, Point.CARTESIAN), new Point(12, -33.3, Point.CARTESIAN), new Point(7, -33.3, Point.CARTESIAN)));
        push1ish.setConstantHeadingInterpolation(0);
        push2 = new Path(new BezierCurve(new Point(2.1, -32, Point.CARTESIAN), new Point(20, -32.3, Point.CARTESIAN), new Point(34, -31, Point.CARTESIAN), new Point(44.6, -32.3, Point.CARTESIAN), new Point(48, -36.9, Point.CARTESIAN)));
        push2ish = new Path(new BezierCurve(new Point(48, -36.9, Point.CARTESIAN), new Point(47.4, -42.3, Point.CARTESIAN), new Point(43.3, -44.3, Point.CARTESIAN), new Point(9.8, -41.2, Point.CARTESIAN)));
        push2.setConstantHeadingInterpolation(0);
        push2ish.setConstantHeadingInterpolation(0);
        push3 = new Path(new BezierCurve(new Point(19.5, -42.2, Point.CARTESIAN), new Point(27, -34, Point.CARTESIAN), new Point(34, -30, Point.CARTESIAN), new Point(42.1, -29, Point.CARTESIAN), new Point(52, -32, Point.CARTESIAN), new Point(53, -35, Point.CARTESIAN)));
        push3.setLinearHeadingInterpolation(0,Math.toRadians(180));
        push3ish = new Path(new BezierCurve(new Point(53, -46.5, Point.CARTESIAN), new Point(40, -46.5, Point.CARTESIAN), new Point(30, -46.5, Point.CARTESIAN), new Point(22, -46.5, Point.CARTESIAN), new Point(15, -46.5, Point.CARTESIAN) , new Point(13, -46.5, Point.CARTESIAN), new Point(11, -46.5, Point.CARTESIAN), new Point(10.2, -46.5, Point.CARTESIAN)));
        push3ish.setConstantHeadingInterpolation(Math.toRadians(180));
        score2 = new Path(new BezierCurve(new Point(8.7, -47.7, Point.CARTESIAN), new Point(11.2, -40.2, Point.CARTESIAN), new Point(12.4, -32.1, Point.CARTESIAN), new Point(13.4, -16.6, Point.CARTESIAN), new Point(16.7, -7.4, Point.CARTESIAN), new Point(22.4, 3.6, Point.CARTESIAN), new Point(26.9, 9.4, Point.CARTESIAN), new Point(29.5, 5.7, Point.CARTESIAN), new Point(29, 13, Point.CARTESIAN)));
        score2.setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(0), .7);
        comeback1 = new Path(new BezierCurve(new Point(28.3, 11.9, Point.CARTESIAN), new Point(23.6, 7.2, Point.CARTESIAN), new Point(18.3, -1.4, Point.CARTESIAN), new Point(18.3, -6.9, Point.CARTESIAN), new Point(18.3, -11.9, Point.CARTESIAN)));
        comeback1ish = new Path(new BezierCurve(new Point(18.3, -11.9, Point.CARTESIAN), new Point(18.3 , -22, Point.CARTESIAN), new Point(9.9, -19.2 , Point.CARTESIAN)));
        comeback1.setConstantHeadingInterpolation(Math.toRadians(180));
        comeback1ish.setConstantHeadingInterpolation(Math.toRadians(180));
        score3 = new Path(new BezierCurve(new Point(10.5, -12.6, Point.CARTESIAN), new Point(13.7, -2, Point.CARTESIAN), new Point(19.3, 9.5, Point.CARTESIAN)));
        score3ish = new Path(new BezierCurve(new Point(19.3, 9.5, Point.CARTESIAN), new Point(24.7, 10.7, Point.CARTESIAN), new Point(28, 15, Point.CARTESIAN)));
        score3.setConstantHeadingInterpolation(Math.toRadians(0));
        score3ish.setConstantHeadingInterpolation(Math.toRadians(0));
        follower.followPath(score1);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */

    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            if (forward == 1) {
                forward = 2;
                follower.followPath(push1);
            } else if(forward == 2){
                forward = 3;
                follower.followPath(push1ish);
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
            }
            else if(forward == 7) {
                forward = 8;
                follower.setPose(new Pose(8.7, -47.7, Math.toRadians(180)));
                follower.followPath(score2);
            }else if(forward == 8) {
                forward = 9;
                follower.followPath(comeback1);
            }
            else if(forward == 9) {
                forward = 10;
                follower.followPath(comeback1ish);
            }else if(forward == 10) {
                forward = 11;
                follower.followPath(score3);
            }
            else if(forward == 11) {
                forward = 12;
                follower.followPath(score3ish);
            }
            else{
                follower.holdPoint(score3ish.getLastControlPoint(), 0);
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
