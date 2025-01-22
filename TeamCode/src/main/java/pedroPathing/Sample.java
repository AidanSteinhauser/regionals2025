package pedroPathing;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Sample", group = "Regionals")
public class Sample extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private Telemetry telemetryA;

    private final Pose firstPose = new Pose(135.65, 39.65, Math.toRadians(270));
    private final Pose secondPose = new Pose(125, 39.65, Math.toRadians(270));
    private final Pose prescorePose = new Pose(125, 19, Math.toRadians(135));
    private final Pose scoringPose = new Pose(128, 16, Math.toRadians(135));
    private final Pose fifthPose = new Pose(125, 23, Math.toRadians(180));
    private final Pose sixthPose = new Pose(125, 12, Math.toRadians(180));
    private final Pose seventhPose = new Pose(100, 30, Math.toRadians(270));
    private PathChain action1, action2, action3, action4, action5, action6, action7, action8, action9, action10;
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
                .addPath(new BezierLine(new Point(secondPose), new Point(prescorePose)))
                .setLinearHeadingInterpolation(secondPose.getHeading(), prescorePose.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prescorePose), new Point(scoringPose)))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), scoringPose.getHeading())
                .build();

        action4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringPose), new Point(fifthPose)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), fifthPose.getHeading())
                .build();

        action5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPose), new Point(prescorePose)))
                .setLinearHeadingInterpolation(fifthPose.getHeading(), prescorePose.getHeading())
                .build();

        action6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prescorePose), new Point(scoringPose)))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), scoringPose.getHeading())
                .build();

        action7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringPose), new Point(sixthPose)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), sixthPose.getHeading())
                .build();

        action8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthPose), new Point(prescorePose)))
                .setLinearHeadingInterpolation(sixthPose.getHeading(), prescorePose.getHeading())
                .build();

        action9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prescorePose), new Point(scoringPose)))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), scoringPose.getHeading())
                .build();

        action10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoringPose), new Point(seventhPose)))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), seventhPose.getHeading())
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
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(prescorePose);
                    sleep(2000);
                    pidgeonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(4130);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(2190);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(4000);
                    follower.followPath(action3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (((!follower.isBusy()) && sliderMotor.getCurrentPosition() > 2170) && pidgeonMotor.getCurrentPosition() > 4100) {
                    follower.holdPoint(scoringPose);
                    pidgeonServo.setPosition(0.9);
                    sleep(2000);
                    releaseServo.setPosition(1);
                    sleep(2000);
                    releaseServo.setPosition(0);
                    sleep(2000);
                    pidgeonServo.setPosition(0);
                    sleep(2000);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(1600);
                    sleep(2000);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(882);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(fifthPose);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(0.5);
                    pidgeonMotor.setTargetPosition(200);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    follower.followPath(action5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(prescorePose);
                    sleep(2000);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(4130);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(2190);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(4000);
                    follower.followPath(action6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (((!follower.isBusy()) && sliderMotor.getCurrentPosition() > 2170) && pidgeonMotor.getCurrentPosition() > 4100) {
                    follower.holdPoint(scoringPose);
                    pidgeonServo.setPosition(0.9);
                    sleep(2000);
                    releaseServo.setPosition(1);
                    sleep(2000);
                    releaseServo.setPosition(0);
                    sleep(2000);
                    pidgeonServo.setPosition(0);
                    sleep(2000);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(1600);
                    sleep(2000);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(882);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(sixthPose);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(0.5);
                    pidgeonMotor.setTargetPosition(200);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    follower.followPath(action8, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.holdPoint(prescorePose);
                    sleep(2000);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(4130);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(2190);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(4000);
                    follower.followPath(action9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (((!follower.isBusy()) && sliderMotor.getCurrentPosition() > 2170) && pidgeonMotor.getCurrentPosition() > 4100) {
                    follower.holdPoint(scoringPose);
                    pidgeonServo.setPosition(0.9);
                    sleep(2000);
                    releaseServo.setPosition(1);
                    sleep(2000);
                    releaseServo.setPosition(0);
                    sleep(2000);
                    pidgeonServo.setPosition(0);
                    sleep(2000);
                    sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    sliderMotor.setPower(1);
                    sliderMotor.setTargetPosition(1600);
                    sleep(2000);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(1);
                    pidgeonMotor.setTargetPosition(882);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action10, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.holdPoint(sixthPose);
                    pidgeonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    pidgeonMotor.setPower(0.5);
                    pidgeonMotor.setTargetPosition(200);
                    pidgeonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(2000);
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
        follower.setStartingPose(firstPose);
        skibidiSlider = hardwareMap.get(DcMotor.class, "skibidiSlider");
        pidgeonMotor = hardwareMap.get(DcMotor.class, "pidgeonMotor");
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        pidgeonServo = hardwareMap.get(Servo.class, "pidgeonServo");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
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