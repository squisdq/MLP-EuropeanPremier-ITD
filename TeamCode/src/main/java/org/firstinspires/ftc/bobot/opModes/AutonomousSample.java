package org.firstinspires.ftc.bobot.opModes;

import static java.lang.Math.PI;

import static org.firstinspires.ftc.bobot.Teleopus.Alliance.RED;

import static org.firstinspires.ftc.bobot.controllers.MasterController.OpMode.AUTONOMOUS_SAMPLE;

import static org.firstinspires.ftc.bobot.opModes.AutonomousSample.MappingMode.GRAB_PICKUP;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSample.MappingMode.INTAKE_SUBMERSIBLE;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSample.MappingMode.SCORE_PICKUP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.util.Direction;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState;
import org.firstinspires.ftc.bobot.controllers.MasterController;
import org.firstinspires.ftc.bobot.controllers.MasterController.IntakeMechanismState;
import org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState;

import org.firstinspires.ftc.bobot.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.bobot.pedroPathing.constants.LConstants;

@Autonomous(group = "0opMode")
public class AutonomousSample extends OpMode {

    private final Pose startPose = new Pose(0.00, 0.00, 0.00);
    private final Pose scorePose = new Pose(-18.00, 6.00, PI / 4);

    private final Pose grabPickup1Pose = new Pose(-9.00, 14.80, PI / 2);
    private final Pose grabPickup2Pose1 = new Pose(-17.50, 14.80, PI / 2);
    private final Pose grabPickup2Pose2 = new Pose(-19.50, 14.80, PI / 2);
    private final Pose grabPickup3Pose = new Pose(-12.20, 22.00, 3 * PI / 4);

    private final Pose parkPose = new Pose(19.55, 55.00, 0.00);
    private final Pose parkCP = new Pose(0.00, 55.00, 0.00);

    private Path scorePreload;
    private Path grabPickup1, scorePickup1;
    private Path grabPickup2FromPickup1, grabPickup2, scorePickup2;
    private Path grabPickup3FromPickup2, grabPickup3, scorePickup3;
    private Path intakeSubmersibleFromPickup3, intakeSubmersible;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = new Path(new BezierLine(new Point(scorePose), new Point(grabPickup1Pose)));
        grabPickup1.setLinearHeadingInterpolation(scorePose.getHeading(), grabPickup1Pose.getHeading());

        scorePickup1 = new Path(new BezierLine(new Point(grabPickup1Pose), new Point(scorePose)));
        scorePickup1.setLinearHeadingInterpolation(grabPickup1Pose.getHeading(), scorePose.getHeading());

        grabPickup2FromPickup1 = new Path(new BezierLine(new Point(grabPickup1Pose), new Point(grabPickup2Pose1)));
        grabPickup2FromPickup1.setConstantHeadingInterpolation(grabPickup1Pose.getHeading());

        grabPickup2 = new Path(new BezierLine(new Point(scorePose), new Point(grabPickup2Pose2)));
        grabPickup2.setLinearHeadingInterpolation(scorePose.getHeading(), grabPickup2Pose2.getHeading());

        scorePickup2 = new Path(new BezierLine(new Point(grabPickup2Pose2), new Point(scorePose)));
        scorePickup2.setLinearHeadingInterpolation(grabPickup2Pose2.getHeading(), scorePose.getHeading());

        grabPickup3FromPickup2 = new Path(new BezierLine(new Point(grabPickup2Pose2), new Point(grabPickup3Pose)));
        grabPickup3FromPickup2.setLinearHeadingInterpolation(grabPickup2Pose2.getHeading(), grabPickup3Pose.getHeading());

        grabPickup3 = new Path(new BezierLine(new Point(scorePose), new Point(grabPickup3Pose)));
        grabPickup3.setLinearHeadingInterpolation(scorePose.getHeading(), grabPickup3Pose.getHeading());

        scorePickup3 = new Path(new BezierLine(new Point(grabPickup3Pose), new Point(scorePose)));
        scorePickup3.setLinearHeadingInterpolation(grabPickup3Pose.getHeading(), scorePose.getHeading());

        intakeSubmersibleFromPickup3 = new Path(new BezierLine(new Point(grabPickup3Pose), new Point(parkPose)));
        intakeSubmersibleFromPickup3.setLinearHeadingInterpolation(grabPickup3Pose.getHeading(), parkPose.getHeading());

        intakeSubmersible = new Path(new BezierCurve(new Point(scorePose), new Point(parkCP), new Point(parkPose)));
        intakeSubmersible.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    private int pathIndex;
    private ElapsedTime pathTimer;
    private boolean ignoreFailure;

    private void setPathIndex(int index) {
        pathIndex = index;
        pathTimer.reset();
        ignoreFailure = true;
    }

    private List<LynxModule> allHubs;
    private double lastTimeStamp;

    private Follower follower;
    private MasterController masterController;

    private MultipleTelemetry multipleTelemetry;
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        masterController = new MasterController(hardwareMap, RED, AUTONOMOUS_SAMPLE);

        pathTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        multipleTelemetry.setMsTransmissionInterval(50);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        buildPaths();
    }

    @Override
    public void start() {
        setPathIndex(0);
        lastTimeStamp = 0;
    }

    private void incrementPathIndex(int increment) {
        setPathIndex(pathIndex + increment);
    }

    private void incrementPathIndex() {
        incrementPathIndex(1);
    }

    private void followPath(double maxSpeed, Path path, int increment) {
        follower.setMaxPower(maxSpeed);
        follower.followPath(path);

        incrementPathIndex(increment);
    }

    private void followPath(double maxSpeed, Path path) {
        followPath(maxSpeed, path, 1);
    }

    private boolean isFollowerCooking() {
        return follower.isBusy();
    }

    private double map(double x, MappingMode mode) {
        if (x < mode.inMin) return Double.NaN;
        if (x > mode.inMax) return Double.NaN;

        double inRange = mode.inMax - mode.inMin;
        double outRange = mode.outMax - mode.outMin;

        return (x - mode.inMin) * outRange / inRange + mode.outMin;
    }

    private void autonomousPathUpdate() {
        if (pathIndex == 0) {
            followPath(0.75, scorePreload);
        } else if (pathIndex == 1) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            followPath(0.75, grabPickup1);
            masterController.proceedSampleScoring();
        } else if (pathIndex == 2) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
        } else if (pathIndex == 3) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 4) {
            if (masterController.getIntakeMechanismState() == IntakeMechanismState.A_TRANSFER2) {
                incrementPathIndex();
                masterController.transitionVerticalSlide(VerticalSlideState.EXTENDED_LONG.position);
            } else if (masterController.getIntakeMechanismState() == IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(0.75, grabPickup2FromPickup1, 3);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RETRACTED.position);
            }
        } else if (pathIndex == 5) {
            if (pathTimer.time() <= 500) return;

            followPath(0.75, scorePickup1);
        } else if (pathIndex == 6) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            followPath(0.75, grabPickup2);
            masterController.proceedSampleScoring();
        } else if (pathIndex == 7) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
        } else if (pathIndex == 8) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 9) {
            if (masterController.getIntakeMechanismState() == IntakeMechanismState.A_TRANSFER2) {
                incrementPathIndex();
                masterController.transitionVerticalSlide(VerticalSlideState.EXTENDED_LONG.position);
            } else if (masterController.getIntakeMechanismState() == IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(0.75, grabPickup3FromPickup2, 3);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RETRACTED.position);
                masterController.rotateElbow(Direction.RIGHT);
            }
        } else if (pathIndex == 10) {
            if (pathTimer.time() <= 500) return;

            followPath(0.75, scorePickup2);
        } else if (pathIndex == 11) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            followPath(0.75, grabPickup3);
            masterController.rotateElbow(Direction.RIGHT);
            masterController.proceedSampleScoring();
        } else if (pathIndex == 12) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
        } else if (pathIndex == 13) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 14) {
            if (masterController.getIntakeMechanismState() == IntakeMechanismState.A_TRANSFER2) {
                incrementPathIndex();
                masterController.transitionVerticalSlide(VerticalSlideState.EXTENDED_LONG.position);
            } else if (masterController.getIntakeMechanismState() == IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(1.00, intakeSubmersibleFromPickup3, 3);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RESET.position);
                masterController.rotateElbow(Direction.LEFT);
            }
        } else if (pathIndex == 15) {
            if (pathTimer.time() <= 500) return;

            followPath(0.75, scorePickup3);
        } else if (pathIndex == 16) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            followPath(1.00, intakeSubmersible);
            masterController.proceedSampleScoring();
        } else if (pathIndex == 17) {
            double mappedMaxPower = map(follower.getCurrentTValue(), INTAKE_SUBMERSIBLE);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking() && !follower.isRobotStuck()) return;

            follower.breakFollowing();
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        follower.update();
        autonomousPathUpdate();

        multipleTelemetry.addData("path index", pathIndex);

        masterController.update();
        masterController.debug(multipleTelemetry);

        double frequency = 1 / period;
        double voltage = voltageSensor.getVoltage();

        multipleTelemetry.addData("frequency", frequency);
        multipleTelemetry.addData("voltage", voltage);

        multipleTelemetry.update();
    }

    public enum MappingMode {
        SCORE_PICKUP(0.65, 0.90, 0.75, 0.45),
        GRAB_PICKUP(0.70, 0.90, 0.75, 0.30),
        INTAKE_SUBMERSIBLE(0.75, 0.90, 1.00, 0.45),
        SCORE_SUBMERSIBLE(0.65, 0.90, 1.00, 0.45);

        final double inMin;
        final double inMax;
        final double outMin;
        final double outMax;

        MappingMode(double inMin, double inMax, double outMin, double outMax) {
            this.inMin = inMin;
            this.inMax = inMax;
            this.outMin = outMin;
            this.outMax = outMax;
        }
    }
}
