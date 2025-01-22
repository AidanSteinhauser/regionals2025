package pedroPathing.OldAutos;


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
@Disabled
@Autonomous(name = "FourSpecimenAutoExactCurved", group = "Meet3")
public class FourSpecimenWednesday extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(135.65, 80.35, Math.toRadians(270));
    private final Pose secondPose = new Pose(105, 76, Math.toRadians(270));
    private final Pose thirdPose = new Pose(82, 120, Math.toRadians(0));
    private final Pose thirdControlPoseOne = new Pose(130, 120, Math.toRadians(0));
    private final Pose thirdControlPoseTuah = new Pose(80, 105, Math.toRadians(0));
    private final Pose fifthPose = new Pose(130, 120, Math.toRadians(0));
    private final Pose sixthPose = new Pose(82, 130, Math.toRadians(0));
    private final Pose sixthControlPoseOne = new Pose(82, 110, Math.toRadians(0));
    private final Pose seventhPose = new Pose(130, 130, Math.toRadians(0));
    private final Pose eighthPose = new Pose(110, 120, Math.toRadians(90));
    private final Pose ninthPose = new Pose(136, 120, Math.toRadians(90));
    private final Pose tenthPose = new Pose(104.5, 74, Math.toRadians(270));
    private final Pose tenthControlPoseOne = new Pose(125, 120, Math.toRadians(270));
    private final Pose tenthControlPoseTuah = new Pose(140, 74, Math.toRadians(270));
    private final Pose eleventhPose = new Pose(125, 120, Math.toRadians(105));
    private final Pose twelfthPose = new Pose(136, 120, Math.toRadians(105));
    private final Pose thirteenthPose = new Pose(105, 72, Math.toRadians(270));
    private final Pose thirteenthControlPoseOne = new Pose(125, 120, Math.toRadians(270));
    private final Pose thirteenthControlPoseTUah = new Pose(140, 72, Math.toRadians(270));
    private final Pose fourteenthPose = new Pose(120, 120, Math.toRadians(90));
    private final Pose fifteenthPose = new Pose(135.5, 120, Math.toRadians(90));
    private final Pose sixteenthPose = new Pose(104.5, 70, Math.toRadians(270));
    private final Pose sixteenthControlPoseOne = new Pose(125, 120, Math.toRadians(270));
    private final Pose sixteenthControlPoseTuah = new Pose(140, 70, Math.toRadians(270));
    private final Pose seventeenthPose = new Pose(120, 130, Math.toRadians(180));

    private PathChain action1, action2, action3, action4, action5, action6, action7, action8, action9, action10, action11, action12, action13, action14, action21, action22, action15, action16, action17, action18, action19, action20;
    private DcMotor skibidiSlider = null;


    public void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPose), new Point(secondPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondPose), new Point(thirdControlPoseOne), new Point(thirdControlPoseTuah), new Point(thirdPose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), fifthPose.getHeading())
                .build();


        action5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fifthPose), new Point(sixthControlPoseOne), new Point(sixthPose)))
                .setLinearHeadingInterpolation(fifthPose.getHeading(), sixthPose.getHeading())
                .build();

        action6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthPose), new Point(seventhPose)))
                .setLinearHeadingInterpolation(sixthPose.getHeading(), seventhPose.getHeading())
                .build();

        action7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventhPose), new Point(eighthPose)))
                .setLinearHeadingInterpolation(seventhPose.getHeading(), eighthPose.getHeading())
                .build();

        action8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eighthPose), new Point(ninthPose)))
                .setLinearHeadingInterpolation(eighthPose.getHeading(), ninthPose.getHeading())
                .build();

        action9 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ninthPose), new Point(tenthControlPoseOne), new Point(tenthControlPoseTuah), new Point(tenthPose)))
                .setLinearHeadingInterpolation(ninthPose.getHeading(), tenthPose.getHeading())
                .build();

        action10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenthPose), new Point(eleventhPose)))
                .setLinearHeadingInterpolation(tenthPose.getHeading(), eleventhPose.getHeading())
                .build();
        action11 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eleventhPose), new Point(twelfthPose)))
                .setLinearHeadingInterpolation(eleventhPose.getHeading(), twelfthPose.getHeading())
                .build();
        action12 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(twelfthPose), new Point(thirteenthControlPoseOne), new Point(thirteenthControlPoseTUah), new Point(thirteenthPose)))
                .setLinearHeadingInterpolation(twelfthPose.getHeading(), thirteenthPose.getHeading())
                .build();
        action13 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirteenthPose), new Point(fourteenthPose)))
                .setLinearHeadingInterpolation(thirteenthPose.getHeading(), fourteenthPose.getHeading())
                .build();
        action14 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourteenthPose), new Point(fifteenthPose)))
                .setLinearHeadingInterpolation(fourteenthPose.getHeading(), fifteenthPose.getHeading())
                .build();
        action15 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fifteenthPose), new Point(sixteenthControlPoseOne), new Point(sixteenthControlPoseTuah), new Point(sixteenthPose)))
                .setLinearHeadingInterpolation(fifteenthPose.getHeading(), sixteenthPose.getHeading())
                .build();
        action16 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixteenthPose), new Point(seventeenthPose)))
                .setLinearHeadingInterpolation(sixteenthPose.getHeading(), seventeenthPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                follower.followPath(action1, true);
                skibidiSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                skibidiSlider.setDirection(DcMotorSimple.Direction.REVERSE);
                skibidiSlider.setPower(1);
                skibidiSlider.setTargetPosition(2100);
                skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.holdPoint(secondPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action3, true);
                        setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(action5, true);
                        setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(action6,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(action7,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action8,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action9,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.holdPoint(tenthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action10,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action11,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action12,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.holdPoint(thirteenthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action13,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action14,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action15,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.holdPoint(sixteenthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action16,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(15);
                }
                break;
            case 15:
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
        telemetry.addData("sillyTime", (sillyTimer.getElapsedTime())/1000);
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
        follower.setStartingPose(firstPose);
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