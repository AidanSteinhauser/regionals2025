package pedroPathing;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "threeSpecBasicVariationTEST", group = "Regionals")
public class threeSpecBasicVariationTEST extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(135.65, 80.35, Math.toRadians(270));
    private final Pose firstPosePreOuttake = new Pose(110, 76, Math.toRadians(270));
    private final Pose secondPoseOuttake = new Pose(107.25, 76, Math.toRadians(270));
    private final Pose thirdPosePrePushControlOne = new Pose(130, 117.5, Math.toRadians(0));
    private final Pose thirdPosePrePushControlTwo = new Pose(78, 102.5, Math.toRadians(0));
    private final Pose thirdPosePrePushOne = new Pose(80, 120, Math.toRadians(0));
    private final Pose fourthPosePostPushOne = new Pose(130, 120, Math.toRadians(0));
    private final Pose fifthPosePrePushTwoControlOne = new Pose(78, 110, Math.toRadians(0));
    private final Pose fifthPosePrePushTwo = new Pose(82, 130, Math.toRadians(0));
    private final Pose sixthPosePostPushTwo = new Pose(130, 130, Math.toRadians(0));
    private final Pose seventhPosePrePushThreeControlOne = new Pose(82, 120, Math.toRadians(0));
    private final Pose seventhPosePrePushThree = new Pose(82, 135.5, Math.toRadians(0));
    private final Pose eighthPosePostPushThree = new Pose(130, 135.5, Math.toRadians(0));
    private final Pose ninthPosePreIntakeOne = new Pose(107, 113, Math.toRadians(90));
    private final Pose tenthPoseIntakeOne = new Pose(137.75, 113, Math.toRadians(90));
    private final Pose eleventhPosePostIntakeOne = new Pose(126.5, 113, Math.toRadians(90));
    private final Pose eleventhPosePostIntakeOneTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose twelfthPosePreOuttakeTwoControlOne = new Pose(135, 72, Math.toRadians(270));
    private final Pose twelfthPosePreOuttakeTwo = new Pose(110, 72, Math.toRadians(270));
    private final Pose thirteenthPoseOuttakeTwo = new Pose(106.5, 72, Math.toRadians(270));
    private final Pose fourteenthPosePreIntakeTwoControlOne = new Pose(130, 74, Math.toRadians(90));
    private final Pose fourteenthPosePreIntakeTwo = new Pose(116, 113, Math.toRadians(90));
    private final Pose fifteenthPoseIntakeTwo = new Pose(137.75, 113, Math.toRadians(90));
    private final Pose sixteenthPosePostIntakeTwo = new Pose(126.5, 113, Math.toRadians(90));
    private final Pose sixteenthPosePostIntakeTwoTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose seventeenthPosePreOuttakeThreeControlOne = new Pose(135, 74, Math.toRadians(270));
    private final Pose seventeenthPosePreOuttakeThree = new Pose(110, 68, Math.toRadians(270));
    private final Pose eighteenthPoseOuttakeThree = new Pose(107.5, 68, Math.toRadians(270));
    private final Pose nineteenthPosePreIntakeThreeControlOne = new Pose(130, 74, Math.toRadians(90));
    private final Pose nineteenthPosePreIntakeThree = new Pose(118, 113, Math.toRadians(95));
    private final Pose twentiethPoseIntakeThree = new Pose(135, 113, Math.toRadians(95));
    private final Pose twentyFirstPostPoseIntakeThree = new Pose(126.5, 113, Math.toRadians(95));
    private final Pose twentyFirstPostPoseIntakeThreeTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose twentySecondPosePreOuttakeFourControlOne = new Pose(135, 74, Math.toRadians(270));
    private final Pose twentySecondPosePreOuttakeFour = new Pose(115, 70, Math.toRadians(270));
    private final Pose twentyThirdPoseOuttakeFour = new Pose(112.25, 70, Math.toRadians(270));
    private final Pose twentyFourthPoseParkControlOne = new Pose(130, 70, Math.toRadians(180));
    private final Pose twentyFourthPosePark = new Pose(130, 130, Math.toRadians(180));

    private PathChain action6PushTwo, actionPARK, action11turn, actionPREINTAKEone, action16Turn, action22PreOuttakeFourth, action23OuttakeFourth, action15IntakeTwo, action21turn, action21PostIntakeThree, action20IntakeThree, action19PreIntakeThree, action18OuttakeThree, action17PreOuttakeThree,action16PostIntakeTwo, action14PreIntakeTwo, action12PreOuttakeTwo, action10IntakeOne, action11PostIntakeOne, action8PushThree, action7CurvedForPushThree, action1PreOuttakeOne, action5CurvedForPushTwo, action2OuttakeOne, action13OuttakeTwo, action3DoubleCurvedPrePush, action4PushOne;
    private DcMotor skibidiSlider = null;


    public void buildPaths() {
        action1PreOuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstPosePreOuttake)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPosePreOuttake.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action2OuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPosePreOuttake), new Point(secondPoseOuttake)))
                .setLinearHeadingInterpolation(firstPosePreOuttake.getHeading(), secondPoseOuttake.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action3DoubleCurvedPrePush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondPoseOuttake), new Point(thirdPosePrePushControlOne), new Point(thirdPosePrePushControlTwo), new Point(thirdPosePrePushOne)))
                .setLinearHeadingInterpolation(secondPoseOuttake.getHeading(), thirdPosePrePushOne.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action4PushOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPosePrePushOne), new Point(fourthPosePostPushOne)))
                .setLinearHeadingInterpolation(thirdPosePrePushOne.getHeading(), fourthPosePostPushOne.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action5CurvedForPushTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fourthPosePostPushOne), new Point(fifthPosePrePushTwoControlOne), new Point(fifthPosePrePushTwo)))
                .setLinearHeadingInterpolation(fourthPosePostPushOne.getHeading(), fifthPosePrePushTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action6PushTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPosePrePushTwo), new Point(sixthPosePostPushTwo)))
                .setLinearHeadingInterpolation(fifthPosePrePushTwo.getHeading(), sixthPosePostPushTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        actionPREINTAKEone = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthPosePostPushOne), new Point(ninthPosePreIntakeOne)))
                .setLinearHeadingInterpolation(fourthPosePostPushOne.getHeading(), ninthPosePreIntakeOne.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action10IntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ninthPosePreIntakeOne), new Point(tenthPoseIntakeOne)))
                .setLinearHeadingInterpolation(ninthPosePreIntakeOne.getHeading(), tenthPoseIntakeOne.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action11PostIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenthPoseIntakeOne), new Point(eleventhPosePostIntakeOne)))
                .setLinearHeadingInterpolation(tenthPoseIntakeOne.getHeading(), eleventhPosePostIntakeOne.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action11turn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eleventhPosePostIntakeOne), new Point(eleventhPosePostIntakeOneTurned)))
                .setLinearHeadingInterpolation(eleventhPosePostIntakeOne.getHeading(), eleventhPosePostIntakeOneTurned.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action12PreOuttakeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(eleventhPosePostIntakeOne), new Point(twelfthPosePreOuttakeTwoControlOne), new Point(twelfthPosePreOuttakeTwo)))
                .setLinearHeadingInterpolation(eleventhPosePostIntakeOne.getHeading(), twelfthPosePreOuttakeTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action13OuttakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twelfthPosePreOuttakeTwo), new Point(thirteenthPoseOuttakeTwo)))
                .setLinearHeadingInterpolation(twelfthPosePreOuttakeTwo.getHeading(), thirteenthPoseOuttakeTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action14PreIntakeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirteenthPoseOuttakeTwo), new Point(fourteenthPosePreIntakeTwoControlOne), new Point(fourteenthPosePreIntakeTwo)))
                .setLinearHeadingInterpolation(thirteenthPoseOuttakeTwo.getHeading(), fourteenthPosePreIntakeTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action15IntakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourteenthPosePreIntakeTwo), new Point(fifteenthPoseIntakeTwo)))
                .setLinearHeadingInterpolation(fourteenthPosePreIntakeTwo.getHeading(), fifteenthPoseIntakeTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action16PostIntakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifteenthPoseIntakeTwo), new Point(sixteenthPosePostIntakeTwo)))
                .setLinearHeadingInterpolation(fifteenthPoseIntakeTwo.getHeading(), sixteenthPosePostIntakeTwo.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action16Turn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixteenthPosePostIntakeTwo), new Point(sixteenthPosePostIntakeTwoTurned)))
                .setLinearHeadingInterpolation(sixteenthPosePostIntakeTwo.getHeading(), sixteenthPosePostIntakeTwoTurned.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action17PreOuttakeThree = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sixteenthPosePostIntakeTwo), new Point(seventeenthPosePreOuttakeThreeControlOne), new Point(seventeenthPosePreOuttakeThree)))
                .setLinearHeadingInterpolation(sixteenthPosePostIntakeTwo.getHeading(), seventeenthPosePreOuttakeThree.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action18OuttakeThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventeenthPosePreOuttakeThree), new Point(eighteenthPoseOuttakeThree)))
                .setLinearHeadingInterpolation(seventeenthPosePreOuttakeThree.getHeading(), eighteenthPoseOuttakeThree.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action19PreIntakeThree = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(eighteenthPoseOuttakeThree), new Point(nineteenthPosePreIntakeThreeControlOne), new Point(nineteenthPosePreIntakeThree)))
                .setLinearHeadingInterpolation(eighteenthPoseOuttakeThree.getHeading(), nineteenthPosePreIntakeThree.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        action20IntakeThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(nineteenthPosePreIntakeThree), new Point(twentiethPoseIntakeThree)))
                .setLinearHeadingInterpolation(nineteenthPosePreIntakeThree.getHeading(), twentiethPoseIntakeThree.getHeading())
                .build();

        action21PostIntakeThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentiethPoseIntakeThree), new Point(twentyFirstPostPoseIntakeThree)))
                .setLinearHeadingInterpolation(twentiethPoseIntakeThree.getHeading(), twentyFirstPostPoseIntakeThree.getHeading())
                .build();

        action22PreOuttakeFourth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(twentyFirstPostPoseIntakeThree), new Point(twentySecondPosePreOuttakeFourControlOne), new Point(twentySecondPosePreOuttakeFour)))
                .setLinearHeadingInterpolation(twentyFirstPostPoseIntakeThree.getHeading(), twentySecondPosePreOuttakeFour.getHeading())
                .build();

        action23OuttakeFourth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twentySecondPosePreOuttakeFour), new Point(twentyThirdPoseOuttakeFour)))
                .setLinearHeadingInterpolation(twentySecondPosePreOuttakeFour.getHeading(), twentyThirdPoseOuttakeFour.getHeading())
                .build();

        actionPARK = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(eighteenthPoseOuttakeThree), new Point(twentyFourthPoseParkControlOne), new Point(twentyFourthPosePark)))
                .setLinearHeadingInterpolation(eighteenthPoseOuttakeThree.getHeading(), twentyFourthPosePark.getHeading())
                .setZeroPowerAccelerationMultiplier(6)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(action1PreOuttakeOne, true);
                    skibidiSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    skibidiSlider.setDirection(DcMotorSimple.Direction.REVERSE);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(action2OuttakeOne, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(secondPoseOuttake);
                    sleep(200);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(650);
                    follower.setMaxPower(0.65);
                    follower.followPath(action3DoubleCurvedPrePush, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.65);
                    follower.followPath(action4PushOne, true);
                    setPathState(5);
                }
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(actionPREINTAKEone, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(action10IntakeOne, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(tenthPoseIntakeOne);
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(200);
                    follower.setMaxPower(0.65);
                    follower.followPath(action11PostIntakeOne, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.65);
                    follower.followPath(action12PreOuttakeTwo, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(action13OuttakeTwo, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.holdPoint(thirteenthPoseOuttakeTwo);
                    sleep(200);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(650);
                    follower.setMaxPower(0.65);
                    follower.followPath(action14PreIntakeTwo, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.6);
                    follower.followPath(action15IntakeTwo, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(fifteenthPoseIntakeTwo);
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(200);
                    follower.setMaxPower(0.65);
                    follower.followPath(action16PostIntakeTwo, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.65);
                    follower.followPath(action17PreOuttakeThree, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.65);
                    follower.followPath(action18OuttakeThree, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.holdPoint(eighteenthPoseOuttakeThree);
                    sleep(200);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(650);
                    follower.setMaxPower(1);
                    follower.followPath(actionPARK, true);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
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
        Drawing.drawDebug(follower);
        telemetry.update();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
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
        follower.setStartingPose(startPose);
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