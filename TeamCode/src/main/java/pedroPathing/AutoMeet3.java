package pedroPathing;


import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "AutoMeet3", group = "Regionals")
public class AutoMeet3 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose zeroPose = new Pose(135.65, 80.35, Math.toRadians(270));
    private final Pose firstPose = new Pose(104.5, 76, Math.toRadians(270));
    private final Pose secondPose = new Pose(82, 120, Math.toRadians(0));
    private final Pose secondPoseControl = new Pose(130, 120, Math.toRadians(0));
    private final Pose secondPoseControlTwo = new Pose(80, 100, Math.toRadians(0));
    private final Pose thirdPose = new Pose(130, 120, Math.toRadians(0));
    private final Pose fourthPose = new Pose(82, 130, Math.toRadians(0));
    private final Pose fourthPoseControl = new Pose(82, 110, Math.toRadians(0));
    private final Pose fifthPose = new Pose(130, 130, Math.toRadians(0));
    private final Pose sixthPose = new Pose(110, 110, Math.toRadians(90));
    private final Pose seventhPose = new Pose(137.5, 110, Math.toRadians(90));
    private final Pose PoseInBetween = new Pose(115, 72, Math.toRadians(270));
    private final Pose eighthPose = new Pose(106 , 72, Math.toRadians(270));
    private final Pose eighthPoseControl = new Pose(125, 70, Math.toRadians(270));
    private final Pose ninthPose = new Pose(125, 120, Math.toRadians(100));
    private final Pose tenthPose = new Pose(133.25, 120, Math.toRadians(100));
    private final Pose PoseInBetween2 = new Pose(115, 68, Math.toRadians(270));
    private final Pose eleventhPose = new Pose(106.5, 68, Math.toRadians(270));
    private final Pose eleventhPoseControl = new Pose(125, 68, Math.toRadians(90));
    private final Pose twelfthPoint = new Pose(122, 120, Math.toRadians(105));
    private final Pose thirteenthPose = new Pose(133, 120, Math.toRadians(105));
    private final Pose thirteenthPoseControl = new Pose(125, 70, Math.toRadians(90));
    private final Pose PoseInBetween3 = new Pose(110, 70, Math.toRadians(270));
    private final Pose fourteenthPose = new Pose(107, 70, Math.toRadians(270));
    private final Pose fifteenthPose = new Pose(120, 130, Math.toRadians(180));

    private PathChain action1, action2, action7point5,action10point5, action13point5, action3, action4, action5, action6, action7, action8, action9, action10, action11, action12, action13, action21, action22, action14, action15, action17, action18, action19, action20;
    private DcMotor skibidiSlider = null;


    public void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(zeroPose), new Point(firstPose)))
                .setLinearHeadingInterpolation(zeroPose.getHeading(), firstPose.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstPose), new Point(secondPoseControl), new Point(secondPoseControlTwo), new Point(secondPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPose), new Point(thirdPose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();


        action4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdPose), new Point(fourthPoseControl), new Point(fourthPose)))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), fourthPose.getHeading())
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(fourthPose.getHeading(), fifthPose.getHeading())
                .build();

        action6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPose), new Point(sixthPose)))
                .setLinearHeadingInterpolation(fifthPose.getHeading(), sixthPose.getHeading())
                .build();

        action7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthPose), new Point(seventhPose)))
                .setLinearHeadingInterpolation(sixthPose.getHeading(), seventhPose.getHeading())
                .build();

        action7point5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventhPose), new Point(PoseInBetween)))
                .setLinearHeadingInterpolation(seventhPose.getHeading(), PoseInBetween.getHeading())
                .build();

        action8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PoseInBetween), new Point(eighthPoseControl), new Point(eighthPose)))
                .setLinearHeadingInterpolation(PoseInBetween.getHeading(), eighthPose.getHeading())
                .build();

        action9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eighthPose), new Point(ninthPose)))
                .setLinearHeadingInterpolation(eighthPose.getHeading(), ninthPose.getHeading())
                .build();
        action10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ninthPose), new Point(tenthPose)))
                .setLinearHeadingInterpolation(ninthPose.getHeading(), tenthPose.getHeading())
                .build();

        action10point5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenthPose), new Point(PoseInBetween2)))
                .setLinearHeadingInterpolation(tenthPose.getHeading(), PoseInBetween2.getHeading())
                .build();

        action11 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PoseInBetween2), new Point(eleventhPoseControl), new Point(eleventhPose)))
                .setLinearHeadingInterpolation(PoseInBetween2.getHeading(), eleventhPose.getHeading())
                .build();
        action12 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eleventhPose), new Point(twelfthPoint)))
                .setLinearHeadingInterpolation(eleventhPose.getHeading(), twelfthPoint.getHeading())
                .build();
        action13 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twelfthPoint), new Point(thirteenthPose)))
                .setLinearHeadingInterpolation(twelfthPoint.getHeading(), thirteenthPose.getHeading())
                .build();

        action13point5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirteenthPose), new Point(PoseInBetween3)))
                .setLinearHeadingInterpolation(thirteenthPose.getHeading(), PoseInBetween3.getHeading())
                .build();

        action14 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseInBetween3), new Point(fourteenthPose)))
                .setLinearHeadingInterpolation(PoseInBetween3.getHeading(), fourteenthPose.getHeading())
                .build();
        action15 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourteenthPose), new Point(fifteenthPose)))
                .setLinearHeadingInterpolation(fourteenthPose.getHeading(), fifteenthPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.8);
                follower.followPath(action1, true);
                skibidiSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                skibidiSlider.setDirection(DcMotorSimple.Direction.REVERSE);
                skibidiSlider.setPower(0.85);
                skibidiSlider.setTargetPosition(2000);
                skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(firstPose);
                    sleep(900);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(650);
                    follower.followPath(action2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(action4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(action5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(action6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    skibidiSlider.setPower(0.85);
                    skibidiSlider.setTargetPosition(2000);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action7point5, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(action8, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(eighthPose);
                    sleep(700);
                    skibidiSlider.setPower(0.85);
                    skibidiSlider.setTargetPosition(700);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(700);
                    follower.followPath(action9, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.holdPoint(ninthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(100);
                    follower.setMaxPower(0.6);
                    follower.followPath(action10, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.85);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2000);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action10point5, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.85);
                    follower.followPath(action11,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.holdPoint(eleventhPose);
                    sleep(900);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(700);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(600);
                    follower.followPath(action12,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action13,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action13point5,true);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(action14,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.holdPoint(fourteenthPose);
                    sleep(500);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(700);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(400);
                    follower.followPath(action15,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        sillyTimer = new Timer();
        skibidiSlider = hardwareMap.get(DcMotor.class, "skibidiSlider");
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(zeroPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
        opmodeTimer.resetTimer();
        sillyTimer.resetTimer();
    }

    @Override
    public void start() {
        sillyTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
