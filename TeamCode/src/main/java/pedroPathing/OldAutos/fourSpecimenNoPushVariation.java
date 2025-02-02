package pedroPathing.OldAutos;


import static android.os.SystemClock.sleep;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "fourSpecimenNoPushVariation", group = "Regionals")
public class fourSpecimenNoPushVariation extends OpMode {
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
    private final Pose ninthPosePreIntakeOne = new Pose(120, 113, Math.toRadians(90));
    private final Pose tenthPoseIntakeOne = new Pose(136.5, 113, Math.toRadians(90));
    private final Pose eleventhPosePostIntakeOne = new Pose(126.5, 113, Math.toRadians(90));
    private final Pose eleventhPosePostIntakeOneTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose twelfthPosePreOuttakeTwoControlOne = new Pose(135, 72, Math.toRadians(270));
    private final Pose twelfthPosePreOuttakeTwo = new Pose(112, 72, Math.toRadians(270));
    private final Pose thirteenthPoseOuttakeTwo = new Pose(110.5, 72, Math.toRadians(270));
    private final Pose fourteenthPosePreIntakeTwoControlOne = new Pose(130, 74, Math.toRadians(90));
    private final Pose fourteenthPosePreIntakeTwo = new Pose(116, 113, Math.toRadians(90));
    private final Pose fifteenthPoseIntakeTwo = new Pose(136.5, 113, Math.toRadians(90));
    private final Pose sixteenthPosePostIntakeTwo = new Pose(126.5, 113, Math.toRadians(90));
    private final Pose sixteenthPosePostIntakeTwoTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose seventeenthPosePreOuttakeThreeControlOne = new Pose(135, 74, Math.toRadians(270));
    private final Pose seventeenthPosePreOuttakeThree = new Pose(115, 68, Math.toRadians(270));
    private final Pose eighteenthPoseOuttakeThree = new Pose(112.5, 68, Math.toRadians(270));
    private final Pose nineteenthPosePreIntakeThreeControlOne = new Pose(130, 74, Math.toRadians(90));
    private final Pose nineteenthPosePreIntakeThree = new Pose(118, 113, Math.toRadians(95));
    private final Pose twentiethPoseIntakeThree = new Pose(135, 113, Math.toRadians(95));
    private final Pose twentyFirstPostPoseIntakeThree = new Pose(126.5, 113, Math.toRadians(95));
    private final Pose twentyFirstPostPoseIntakeThreeTurned = new Pose(126.5, 113, Math.toRadians(270));
    private final Pose twentySecondPosePreOuttakeFourControlOne = new Pose(135, 74, Math.toRadians(270));
    private final Pose twentySecondPosePreOuttakeFour = new Pose(115, 70, Math.toRadians(270));
    private final Pose twentyThirdPoseOuttakeFour = new Pose(112, 70, Math.toRadians(270));
    private final Pose twentyFourthPoseParkControlOne = new Pose(130, 70, Math.toRadians(180));
    private final Pose twentyFourthPosePark = new Pose(130, 130, Math.toRadians(180));

    private PathChain action6PushTwo, action24Park, action11turn, action9PreIntakeOne, action16Turn, action22PreOuttakeFourth, action23OuttakeFourth, action15IntakeTwo, action21turn, action21PostIntakeThree, action20IntakeThree, action19PreIntakeThree, action18OuttakeThree, action17PreOuttakeThree,action16PostIntakeTwo, action14PreIntakeTwo, action12PreOuttakeTwo, action10IntakeOne, action11PostIntakeOne, action8PushThree, action7CurvedForPushThree, action1PreOuttakeOne, action5CurvedForPushTwo, action2OuttakeOne, action13OuttakeTwo, action3DoubleCurvedPrePush, action4PushOne;
    private DcMotor skibidiSlider = null;


    public void buildPaths() {
        action1PreOuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstPosePreOuttake)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPosePreOuttake.getHeading())
                .build();

        action2OuttakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPosePreOuttake), new Point(secondPoseOuttake)))
                .setLinearHeadingInterpolation(firstPosePreOuttake.getHeading(), secondPoseOuttake.getHeading())
                .build();

        action3DoubleCurvedPrePush = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondPoseOuttake), new Point(thirdPosePrePushControlOne), new Point(thirdPosePrePushControlTwo), new Point(thirdPosePrePushOne)))
                .setLinearHeadingInterpolation(secondPoseOuttake.getHeading(), thirdPosePrePushOne.getHeading())
                .build();

        action4PushOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPosePrePushOne), new Point(fourthPosePostPushOne)))
                .setLinearHeadingInterpolation(thirdPosePrePushOne.getHeading(), fourthPosePostPushOne.getHeading())
                .build();

        action5CurvedForPushTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(fourthPosePostPushOne), new Point(fifthPosePrePushTwoControlOne), new Point(fifthPosePrePushTwo)))
                .setLinearHeadingInterpolation(fourthPosePostPushOne.getHeading(), fifthPosePrePushTwo.getHeading())
                .build();

        action6PushTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPosePrePushTwo), new Point(sixthPosePostPushTwo)))
                .setLinearHeadingInterpolation(fifthPosePrePushTwo.getHeading(), sixthPosePostPushTwo.getHeading())
                .build();

        action9PreIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthPosePostPushTwo), new Point(ninthPosePreIntakeOne)))
                .setLinearHeadingInterpolation(sixthPosePostPushTwo.getHeading(), ninthPosePreIntakeOne.getHeading())
                .build();

        action10IntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ninthPosePreIntakeOne), new Point(tenthPoseIntakeOne)))
                .setLinearHeadingInterpolation(ninthPosePreIntakeOne.getHeading(), tenthPoseIntakeOne.getHeading())
                .build();

        action11PostIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(tenthPoseIntakeOne), new Point(eleventhPosePostIntakeOne)))
                .setLinearHeadingInterpolation(tenthPoseIntakeOne.getHeading(), eleventhPosePostIntakeOne.getHeading())
                .build();

        action11turn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(eleventhPosePostIntakeOne), new Point(eleventhPosePostIntakeOneTurned)))
                .setLinearHeadingInterpolation(eleventhPosePostIntakeOne.getHeading(), eleventhPosePostIntakeOneTurned.getHeading())
                .build();

        action12PreOuttakeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(eleventhPosePostIntakeOne), new Point(twelfthPosePreOuttakeTwoControlOne), new Point(twelfthPosePreOuttakeTwo)))
                .setLinearHeadingInterpolation(eleventhPosePostIntakeOne.getHeading(), twelfthPosePreOuttakeTwo.getHeading())
                .build();

        action13OuttakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(twelfthPosePreOuttakeTwo), new Point(thirteenthPoseOuttakeTwo)))
                .setLinearHeadingInterpolation(twelfthPosePreOuttakeTwo.getHeading(), thirteenthPoseOuttakeTwo.getHeading())
                .build();

        action14PreIntakeTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirteenthPoseOuttakeTwo), new Point(fourteenthPosePreIntakeTwoControlOne), new Point(fourteenthPosePreIntakeTwo)))
                .setLinearHeadingInterpolation(thirteenthPoseOuttakeTwo.getHeading(), fourteenthPosePreIntakeTwo.getHeading())
                .build();

        action15IntakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourteenthPosePreIntakeTwo), new Point(fifteenthPoseIntakeTwo)))
                .setLinearHeadingInterpolation(fourteenthPosePreIntakeTwo.getHeading(), fifteenthPoseIntakeTwo.getHeading())
                .build();

        action16PostIntakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifteenthPoseIntakeTwo), new Point(sixteenthPosePostIntakeTwo)))
                .setLinearHeadingInterpolation(fifteenthPoseIntakeTwo.getHeading(), sixteenthPosePostIntakeTwo.getHeading())
                .build();

        action16Turn = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixteenthPosePostIntakeTwo), new Point(sixteenthPosePostIntakeTwoTurned)))
                .setLinearHeadingInterpolation(sixteenthPosePostIntakeTwo.getHeading(), sixteenthPosePostIntakeTwoTurned.getHeading())
                .build();

        action17PreOuttakeThree = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sixteenthPosePostIntakeTwo), new Point(seventeenthPosePreOuttakeThreeControlOne), new Point(seventeenthPosePreOuttakeThree)))
                .setLinearHeadingInterpolation(sixteenthPosePostIntakeTwo.getHeading(), seventeenthPosePreOuttakeThree.getHeading())
                .build();

        action18OuttakeThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(seventeenthPosePreOuttakeThree), new Point(eighteenthPoseOuttakeThree)))
                .setLinearHeadingInterpolation(seventeenthPosePreOuttakeThree.getHeading(), eighteenthPoseOuttakeThree.getHeading())
                .build();

        action19PreIntakeThree = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(eighteenthPoseOuttakeThree), new Point(nineteenthPosePreIntakeThreeControlOne), new Point(nineteenthPosePreIntakeThree)))
                .setLinearHeadingInterpolation(eighteenthPoseOuttakeThree.getHeading(), nineteenthPosePreIntakeThree.getHeading())
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

        action24Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(twentyThirdPoseOuttakeFour), new Point(twentyFourthPoseParkControlOne), new Point(twentyFourthPosePark)))
                .setLinearHeadingInterpolation(twentyThirdPoseOuttakeFour.getHeading(), twentyFourthPosePark.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
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
                  follower.setMaxPower(0.8);
                    follower.followPath(action2OuttakeOne, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(secondPoseOuttake);
                    sleep(175);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.setMaxPower(0.8);
                    follower.followPath(action3DoubleCurvedPrePush, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.followPath(action4PushOne, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(action5CurvedForPushTwo, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(action6PushTwo, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(action9PreIntakeOne, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);

                    follower.followPath(action10IntakeOne, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(tenthPoseIntakeOne);
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(175);
                    follower.setMaxPower(0.8);
                    follower.followPath(action11PostIntakeOne, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action12PreOuttakeTwo, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                  follower.setMaxPower(0.8);
                    follower.followPath(action13OuttakeTwo, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(thirteenthPoseOuttakeTwo);
                    sleep(175);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action14PreIntakeTwo, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.6);

                    follower.followPath(action15IntakeTwo, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.holdPoint(fifteenthPoseIntakeTwo);
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(175);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action16PostIntakeTwo, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action17PreOuttakeThree, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                  follower.setMaxPower(0.8);
                    follower.followPath(action18OuttakeThree, true);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.holdPoint(eighteenthPoseOuttakeThree);
                    sleep(175);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action19PreIntakeThree, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.6);

                    follower.followPath(action20IntakeThree, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.holdPoint(twentiethPoseIntakeThree);
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(175);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action21PostIntakeThree, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    skibidiSlider.setPower(0.9);
                    skibidiSlider.setTargetPosition(2050);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    follower.setMaxPower(0.8);;
                    follower.followPath(action22PreOuttakeFourth, true);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                  follower.setMaxPower(0.8);
                    follower.followPath(action23OuttakeFourth, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    follower.holdPoint(twentyThirdPoseOuttakeFour);
                    sleep(175);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    follower.setMaxPower(1);;
                    follower.followPath(action24Park, true);
                    skibidiSlider.setPower(0.8);
                    skibidiSlider.setTargetPosition(0);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPathState(23);
                }
            case 23:
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
        telemetry.update();

        Drawing.drawDebug(follower);
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