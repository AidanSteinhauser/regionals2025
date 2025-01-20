package pedroPathing.OldAutos;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name = "twoSpecV2", group = "Meet3")
public class twoSpecV2 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(135.65, 72, Math.toRadians(270));
    private final Pose secondPose = new Pose(110, 72, Math.toRadians(270));
    private final Pose thirdPose = new Pose(105, 72, Math.toRadians(270));
    private final Pose fourthPose = new Pose(130, 120, Math.toRadians(90));
    private final Pose fifthPose = new Pose(135.65, 120, Math.toRadians(90));
    private final Pose sixthPose = new Pose(133, 120, Math.toRadians(90));
    private final Pose seventhPose = new Pose(110, 74, Math.toRadians(270));
    private final Pose eighthPose = new Pose(105, 74, Math.toRadians(270));
    private final Pose ninthPose = new Pose(130, 130, Math.toRadians(180));

    private PathChain action2, action4, action6, action3, action5, action7, action8, action1;
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
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(action1, true);
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
            case 8:
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
        telemetry.addData("sillyTime", sillyTimer.getElapsedTime());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        sillyTimer = new Timer();
        follower = new Follower(hardwareMap);
        skibidiSlider = hardwareMap.get(DcMotor.class, "skibidiSlider");
        opmodeTimer.resetTimer();

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