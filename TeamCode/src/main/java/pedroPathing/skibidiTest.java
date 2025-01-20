package pedroPathing;


import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name = "skibidiTest", group = "Meet3")
public class skibidiTest extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer sillyTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final Pose firstPose = new Pose(110, 80.35, Math.toRadians(0));
    private final Pose secondPose = new Pose(130, 80.35, Math.toRadians(0));
    private final Pose thirdPose = new Pose(135, 80.35, Math.toRadians(0));


    private PathChain action1, action2;
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

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(action1, true);
                setPathState(1);
                skibidiSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                if(!follower.isBusy()) {
                    skibidiSlider.setDirection(DcMotorSimple.Direction.REVERSE);
                    skibidiSlider.setPower(0.5);
                    skibidiSlider.setTargetPosition(500);
                    skibidiSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                        setPathState(-1);
                        break;
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