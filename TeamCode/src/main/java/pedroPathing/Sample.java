package pedroPathing;


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
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "Sample", group = "Meet3")
public class Sample extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(135.65, 39.65, Math.toRadians(270));
    private final Pose secondPose = new Pose(130, 39.65, Math.toRadians(135));
    private final Pose thirdPose = new Pose(127, 17, Math.toRadians(167));
    private final Pose fourthPose = new Pose(127, 17, Math.toRadians(180));
    private final Pose fifthPose = new Pose(127, 17, Math.toRadians(193));
    private final Pose sixthPose = new Pose(80, 30, Math.toRadians(0));
    private final Pose seventhPose = new Pose(80, 9, Math.toRadians(0));
    private final Pose eighthPose = new Pose(130, 9, Math.toRadians(0));

    private PathChain action1, action2, action3, action4, action5, action6, action7;
    private DcMotor skibidiSlider = null;
    private DcMotor pidgeonMotor = null;
    private DcMotor sliderMotor = null;
    private Servo releaseServo = null;
    private Servo pidgeonServo = null;


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
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.5);
                follower.followPath(action1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(action2, true);
                    setPathState(2);
                    pidgeonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(0.5);
                    pidgeonMotor.setTargetPosition(4130);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(0.5);
                    sliderMotor.setTargetPosition(1550);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case 2:
                if ((!follower.isBusy()) && sliderMotor.getCurrentPosition() > 1540) {
                    follower.holdPoint(thirdPose);
                    sleep(300);
                    pidgeonServo.setPosition(1);
                    sleep(500);
                    releaseServo.setPosition(1);
                    sleep(700);
                    releaseServo.setPosition(0);
                    sleep(700);
                    pidgeonServo.setPosition(0);
                    sleep(500);
                    pidgeonMotor.setPower(0.5);
                    pidgeonMotor.setTargetPosition(890);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(300);
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
                    follower.followPath(action7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
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
        telemetry.addData("sillyTime", (sillyTimer.getElapsedTime()) / 1000);
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
        skibidiSlider = hardwareMap.get(DcMotor.class, "skibidiSlider");
        pidgeonMotor = hardwareMap.get(DcMotor.class, "pidgeonMotor");
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        pidgeonServo = hardwareMap.get(Servo.class, "pidgeonServo");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
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