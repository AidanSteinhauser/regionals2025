package pedroPathing.OldAutos;


import static android.os.SystemClock.sleep;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "FiveSpecimenAutoExact", group = "Meet3")
public class FiveSpecimenAutoExact extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(135.65, 80.35, Math.toRadians(270));
    private final Pose secondPose = new Pose(112, 76, Math.toRadians(270));
    private final Pose thirdPose = new Pose(107, 76, Math.toRadians(270));
    private final Pose fourthPose = new Pose(116, 109, Math.toRadians(270));
    private final Pose fifthPose = new Pose(82, 109, Math.toRadians(0));
    private final Pose sixthPose = new Pose(82, 119, Math.toRadians(0));
    private final Pose seventhPose = new Pose(133, 119, Math.toRadians(0));
    private final Pose eighthPose = new Pose(82, 119, Math.toRadians(0));
    private final Pose ninthPose = new Pose(82, 130, Math.toRadians(0));
    private final Pose tenthPose = new Pose(133, 130, Math.toRadians(0));
    private final Pose tentuahPose = new Pose(82, 130, Math.toRadians(0));
    private final Pose tenthreeahPose = new Pose(82, 135, Math.toRadians(0));
    private final Pose tenfourahPose = new Pose(133, 135, Math.toRadians(0));
    private final Pose eleventhPose = new Pose(110, 120, Math.toRadians(90));
    private final Pose twelfthPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose thirteenthPose = new Pose(112, 74, Math.toRadians(270));
    private final Pose fourteenthPose = new Pose(107, 74, Math.toRadians(270));
    private final Pose fifteenthPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose sixteenthPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose seventeenthPose = new Pose(112, 72, Math.toRadians(270));
    private final Pose eighteenthPose = new Pose(107, 72, Math.toRadians(270));
    private final Pose ninteenthPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose twentiethPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose twentyfirstPose = new Pose(112, 70, Math.toRadians(270));
    private final Pose twentysecondPose = new Pose(107, 70, Math.toRadians(270));
    private final Pose twentythirdPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose twentyfourthPose = new Pose(135, 120, Math.toRadians(90));
    private final Pose twentyfifthPose = new Pose(112, 68, Math.toRadians(270));
    private final Pose twentysixthPose = new Pose(107, 68, Math.toRadians(270));
    private final Pose twentyseventhPose = new Pose(130, 130, Math.toRadians(180));

    private PathChain action1, action2, action3, action4, action5, action6, action7, action29, action26, action27, action28, action25, action23, action24, action8, action9, action10, action11, action12, action13, action14, action21, action22, action15, action16, action17, action18, action19, action20;
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
                .addPath(new BezierLine(new Point(tenthPose), new Point(tentuahPose)))
                .setLinearHeadingInterpolation(tenthPose.getHeading(), tentuahPose.getHeading())
                .build();
        action11 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tentuahPose), new Point(tenthreeahPose)))
                .setLinearHeadingInterpolation(tentuahPose.getHeading(), tenthreeahPose.getHeading())
                .build();
        action12 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenthreeahPose), new Point(tenfourahPose)))
                .setLinearHeadingInterpolation(tenthreeahPose.getHeading(), tenfourahPose.getHeading())
                .build();
        action13 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenfourahPose), new Point(eleventhPose)))
                .setLinearHeadingInterpolation(tenfourahPose.getHeading(), eleventhPose.getHeading())
                .build();
        action14 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eleventhPose), new Point(twelfthPose)))
                .setLinearHeadingInterpolation(eleventhPose.getHeading(), twelfthPose.getHeading())
                .build();
        action15 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twelfthPose), new Point(thirteenthPose)))
                .setLinearHeadingInterpolation(twelfthPose.getHeading(), thirteenthPose.getHeading())
                .build();
        action16 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirteenthPose), new Point(fourteenthPose)))
                .setLinearHeadingInterpolation(thirteenthPose.getHeading(), fourteenthPose.getHeading())
                .build();
        action17 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourteenthPose), new Point(fifteenthPose)))
                .setLinearHeadingInterpolation(fourteenthPose.getHeading(), fifteenthPose.getHeading())
                .build();
        action18 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifteenthPose), new Point(sixteenthPose)))
                .setLinearHeadingInterpolation(fifteenthPose.getHeading(), sixteenthPose.getHeading())
                .build();
        action19 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixteenthPose), new Point(seventeenthPose)))
                .setLinearHeadingInterpolation(sixteenthPose.getHeading(), seventeenthPose.getHeading())
                .build();
        action20 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventeenthPose), new Point(eighteenthPose)))
                .setLinearHeadingInterpolation(seventeenthPose.getHeading(), eighteenthPose.getHeading())
                .build();
        action21 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eighteenthPose), new Point(ninteenthPose)))
                .setLinearHeadingInterpolation(eighteenthPose.getHeading(), ninteenthPose.getHeading())
                .build();
        action22 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ninteenthPose), new Point(twentiethPose)))
                .setLinearHeadingInterpolation(ninteenthPose.getHeading(), twentiethPose.getHeading())
                .build();
        action23 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentiethPose), new Point(twentyfirstPose)))
                .setLinearHeadingInterpolation(twentiethPose.getHeading(), twentyfirstPose.getHeading())
                .build();

        action24 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentyfirstPose), new Point(twentysecondPose)))
                .setLinearHeadingInterpolation(twentyfirstPose.getHeading(), twentysecondPose.getHeading())
                .build();
        action25 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentysecondPose), new Point(twentythirdPose)))
                .setLinearHeadingInterpolation(twentysecondPose.getHeading(), twentythirdPose.getHeading())
                .build();
        action26 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentythirdPose), new Point(twentyfourthPose)))
                .setLinearHeadingInterpolation(twentythirdPose.getHeading(), twentyfourthPose.getHeading())
                .build();
        action27 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentyfourthPose), new Point(twentyfifthPose)))
                .setLinearHeadingInterpolation(twentyfourthPose.getHeading(), twentyfifthPose.getHeading())
                .build();
        action28 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentyfifthPose), new Point(twentysixthPose)))
                .setLinearHeadingInterpolation(twentyfifthPose.getHeading(), twentysixthPose.getHeading())
                .build();
        action29 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentysixthPose), new Point(twentyseventhPose)))
                .setLinearHeadingInterpolation(twentysixthPose.getHeading(), twentyseventhPose.getHeading())
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
                        follower.followPath(action2, true);
                        setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.holdPoint(thirdPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action3, true);
                        setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                    follower.setMaxPower(0.6);
                    follower.followPath(action14,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                    follower.holdPoint(fourteenthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action17,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action18,true);
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action19,true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    follower.followPath(action20);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.holdPoint(eighteenthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action21,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action22,true);
                    setPathState(22);
                }
                break;
            case 22:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action23,true);
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    follower.followPath(action24,true);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy()) {
                    follower.holdPoint(twentysecondPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action25,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action26,true);
                    setPathState(26);
                }
                break;
            case 26:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(2200);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action27,true);
                    setPathState(27);
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    follower.followPath(action28,true);
                    setPathState(28);
                }
                break;
            case 28:
                if(!follower.isBusy()) {
                    follower.holdPoint(twentysixthPose);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.followPath(action29,true);
                    skibidiSlider.setPower(1);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(29);
                }
                break;
            case 29:
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