package pedroPathing.OldAutos;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "SiSpecV1", group = "Meet3")
public class SiSpecV1 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(135.65, 80.35, Math.toRadians(270));
    private final Pose secondPose = new Pose(115, 76, Math.toRadians(270));
    private final Pose thirdPose = new Pose(108, 76, Math.toRadians(270));
    private final Pose fourthPose = new Pose(116, 107, Math.toRadians(270));
    private final Pose fifthPose = new Pose(87, 107, Math.toRadians(0));
    private final Pose sixthPose = new Pose(87, 120, Math.toRadians(0));
    private final Pose seventhPose = new Pose(130, 120, Math.toRadians(0));
    private final Pose eighthPose = new Pose(87, 120, Math.toRadians(0));
    private final Pose ninthPose = new Pose(87, 130, Math.toRadians(0));
    private final Pose tenthPose = new Pose(130, 130, Math.toRadians(0));
    private final Pose eleventhPose = new Pose(110, 120, Math.toRadians(90));
    private final Pose twelfthPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose thirteenthPose = new Pose(110, 74, Math.toRadians(270));
    private final Pose fourteenthPose = new Pose(106, 74, Math.toRadians(270));
    private final Pose fifteenthPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose sixteenthPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose seventeenthPose = new Pose(110, 72, Math.toRadians(270));
    private final Pose eighteenthPose = new Pose(106, 72, Math.toRadians(270));
    private final Pose ninteenthPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose twentiethPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose twentyfirstPose = new Pose(110, 70, Math.toRadians(270));
    private final Pose twentysecondPose = new Pose(105, 70, Math.toRadians(270));
    private final Pose twentythirdPose = new Pose(130, 130, Math.toRadians(180));

    private PathChain action1, action2, action3, action4, action5, action6, action7, action8, action9, action10, action11, action12, action13, action14, action21, action22, action15, action16, action17, action18, action19, action20;
    private DcMotor skibidiSlider = null;


    public void buildPaths() {
        action1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPose), new Point(secondPose)))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPose), new Point(thirdPose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPose), new Point(fourthPose)))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), fourthPose.getHeading())
                .build();

        action4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(fourthPose.getHeading(), fifthPose.getHeading())
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPose), new Point(sixthPose)))
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
                .addPath(new BezierLine(new Point(ninthPose), new Point(tenthPose)))
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
                .addPath(new BezierLine(new Point(twelfthPose), new Point(thirteenthPose)))
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
                .addPath(new BezierLine(new Point(fifteenthPose), new Point(sixteenthPose)))
                .setLinearHeadingInterpolation(fifteenthPose.getHeading(), sixteenthPose.getHeading())
                .build();
        action16 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixteenthPose), new Point(seventeenthPose)))
                .setLinearHeadingInterpolation(sixteenthPose.getHeading(), seventeenthPose.getHeading())
                .build();
        action17 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventeenthPose), new Point(eighteenthPose)))
                .setLinearHeadingInterpolation(seventeenthPose.getHeading(), eighteenthPose.getHeading())
                .build();
        action18 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eighteenthPose), new Point(ninteenthPose)))
                .setLinearHeadingInterpolation(eighteenthPose.getHeading(), ninteenthPose.getHeading())
                .build();
        action19 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ninteenthPose), new Point(twentiethPose)))
                .setLinearHeadingInterpolation(ninteenthPose.getHeading(), twentiethPose.getHeading())
                .build();
        action20 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentiethPose), new Point(twentyfirstPose)))
                .setLinearHeadingInterpolation(twentiethPose.getHeading(), twentyfirstPose.getHeading())
                .build();

        action21 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentyfirstPose), new Point(twentysecondPose)))
                .setLinearHeadingInterpolation(twentyfirstPose.getHeading(), twentysecondPose.getHeading())
                .build();
        action22 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentysecondPose), new Point(twentythirdPose)))
                .setLinearHeadingInterpolation(twentysecondPose.getHeading(), twentythirdPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                        follower.followPath(action2, true);
                        setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(action3, true);
                        setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(action4, true);
                        setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(action5,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(action6,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(action7,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(action8,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(action9,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(action10,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(action11,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(action12,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(action13,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(action14,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(action15,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(action16,true);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(action17,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(action18,true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.followPath(action19,true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    follower.followPath(action20,true);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(action21,true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.followPath(action22,true);
                    setPathState(22);
                }
                break;
            case 22:
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
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("acceleration", follower.getAcceleration());
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