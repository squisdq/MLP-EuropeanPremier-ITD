package org.firstinspires.ftc.bobot.opModes;

import static java.lang.Math.PI;

import static org.firstinspires.ftc.bobot.Teleopus.Alliance.RED;

import static org.firstinspires.ftc.bobot.controllers.MasterController.OpMode.AUTONOMOUS_SPECIMEN;

import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.GRAB_PICKUP;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.GRAB_PICKUP1;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.GRAB_SPECIMEN;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.PARK;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.SCORE_PRELOAD;
import static org.firstinspires.ftc.bobot.opModes.AutonomousSpecimen.MappingMode.SCORE_SPECIMEN;

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

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState;
import org.firstinspires.ftc.bobot.controllers.MasterController;

import org.firstinspires.ftc.bobot.controllers.MasterController.OuttakeMechanismState;

import org.firstinspires.ftc.bobot.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.bobot.pedroPathing.constants.LConstants;

@Autonomous(group = "0opMode")
public class AutonomousSpecimen extends OpMode {

    private final Pose startPose = new Pose(0.00, 0.00, 0.00);
    private final Pose scorePose = new Pose(35.50, 4.00, 0.00);

    private final Pose grabPickup1CP = new Pose(13.50, 0.00, 0.00);
    private final Pose grabPickup1Pose = new Pose(13.50, -48.00, 0.00);
    private final Pose grabPickup2Pose = new Pose(13.50, -58.00, 0.00);
    private final Pose grabPickup3Pose = new Pose(20.00, -52.00, -PI / 4);
    private final Pose dropPickup3Pose = new Pose(10.00, -58.00, PI / 4);
    
    private final Pose grabFromPickup3CP = new Pose(13.50, -45.00, 0.00);
    private final Pose grabAndScoreCP1 = new Pose(13.50, -45.00, 0.00);
    private final Pose grabAndScoreCP2 = new Pose(13.50, 0.00, 0.00);
    private final Pose grabPose = new Pose(0.00, -45.00, 0);
    
    private final Pose parkPose = new Pose(6.10, -45.00, 0.00);
    private final Pose parkCP = new Pose(6.10, 0.00, 0.00);

    private Path scorePreload;
    private Path grabPickup1, grabPickup2, grabPickup3;
    private Path dropPickup3;
    private Path grabSpecimen1FromPickup3, grabSpecimen1;

    private final ArrayList<Path> scoreSpecimen = new ArrayList<>();
    private final ArrayList<Path> grabSpecimen = new ArrayList<>();

    private Path park;

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        grabPickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(grabPickup1CP), new Point(grabPickup1Pose)));
        grabPickup1.setConstantHeadingInterpolation(scorePose.getHeading());

        grabPickup2 = new Path(new BezierLine(new Point(grabPickup1Pose), new Point(grabPickup2Pose)));
        grabPickup2.setConstantHeadingInterpolation(grabPickup1Pose.getHeading());

        grabPickup3 = new Path(new BezierLine(new Point(grabPickup2Pose), new Point(grabPickup3Pose)));
        grabPickup3.setLinearHeadingInterpolation(grabPickup2Pose.getHeading(), grabPickup3Pose.getHeading());

        dropPickup3 = new Path(new BezierLine(new Point(grabPickup3Pose), new Point(dropPickup3Pose)));
        dropPickup3.setLinearHeadingInterpolation(grabPickup3Pose.getHeading(), dropPickup3Pose.getHeading());

        grabSpecimen1FromPickup3 = new Path(new BezierCurve(new Point(grabPickup3Pose), new Point(grabFromPickup3CP), new Point(grabPose)));
        grabSpecimen1FromPickup3.setLinearHeadingInterpolation(grabPickup3Pose.getHeading(), grabPose.getHeading());

        grabSpecimen1 = new Path(new BezierCurve(new Point(dropPickup3Pose), new Point(grabFromPickup3CP), new Point(grabPose)));
        grabSpecimen1.setLinearHeadingInterpolation(dropPickup3Pose.getHeading(), grabPose.getHeading());

        for (int counter = 0; counter < 4; counter++) {
            Pose transitionedScorePose = new Pose(scorePose.getX(), scorePose.getY() - 1.25 * counter);

            scoreSpecimen.add(counter, new Path(new BezierCurve(new Point(grabPose), new Point(grabAndScoreCP1), new Point(grabAndScoreCP2), new Point(transitionedScorePose))));
            scoreSpecimen.get(counter).setConstantHeadingInterpolation(grabPose.getHeading());

            if (counter == 3) {
                park = new Path(new BezierCurve(new Point(transitionedScorePose), new Point(parkCP), new Point(parkPose)));
                park.setConstantHeadingInterpolation(transitionedScorePose.getHeading());

                break;
            }

            grabSpecimen.add(counter, new Path(new BezierCurve(new Point(transitionedScorePose), new Point(grabAndScoreCP2), new Point(grabAndScoreCP1), new Point(grabPose))));
            grabSpecimen.get(counter).setConstantHeadingInterpolation(transitionedScorePose.getHeading());
        }
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

        masterController = new MasterController(hardwareMap, RED, AUTONOMOUS_SPECIMEN);

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
            followPath(1.00, scorePreload);
        } else if (pathIndex == 1) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_PRELOAD);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking() && !follower.isRobotStuck()) return;

            followPath(1.00, grabPickup1);
            masterController.proceedSpecimenScoring();
        } else if (pathIndex == 2) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP1);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
        } else if (pathIndex == 3) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 4) {
            if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.A_TRANSFER2) {
                followPath(0.75, grabPickup2);
            } else if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(0.75, grabPickup2);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RETRACTED.position);
            }
        } else if (pathIndex == 5) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
            if (masterController.getOuttakeMechanismState() == OuttakeMechanismState.I_SAMPLE_OUTTAKE) masterController.proceedSampleScoring();
        } else if (pathIndex == 6) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 7) {
            if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.A_TRANSFER2) {
                incrementPathIndex();
            } else if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(0.75, grabPickup3, 2);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RETRACTED.position);
                masterController.rotateElbow(Direction.LEFT);
            }
        } else if (pathIndex == 8) {
            if (pathTimer.time() >= 750 && masterController.getOuttakeMechanismState() == OuttakeMechanismState.I_SAMPLE_OUTTAKE) {
                masterController.proceedSampleScoring();
            } else if (pathTimer.time() >= 1000) {
                followPath(0.75, grabPickup3);
                masterController.rotateElbow(Direction.LEFT);
            }
        } else if (pathIndex == 9) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking()) return;

            incrementPathIndex();
            masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_MEDIUM.position);
        } else if (pathIndex == 10) {
            if (masterController.isHorizontalSlideCooking()) return;

            incrementPathIndex();
            masterController.proceedSampleScoring();
        } else if (pathIndex == 11) {
            if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.A_TRANSFER2) {
                followPath(0.75, dropPickup3);
            } else if (masterController.getIntakeMechanismState() == MasterController.IntakeMechanismState.I_INTAKE) {
                if (ignoreFailure) {
                    ignoreFailure = false;
                    masterController.proceedSampleScoring();

                    return;
                }

                followPath(0.75, grabSpecimen1FromPickup3, 2);
                masterController.transitionHorizontalSlide(HorizontalSlideState.RESET.position);
                masterController.rotateElbow(Direction.RIGHT);
                masterController.proceedSpecimenScoring();
            }
        } else if (pathIndex == 12) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_PICKUP);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking() && !follower.isRobotStuck()) return;

            followPath(1.00, grabSpecimen1);
            masterController.proceedSpecimenScoring();
        } else if (pathIndex >= 13 && pathIndex <= 22 && pathIndex % 3 == 1) {
            double mappedMaxPower = map(follower.getCurrentTValue(), GRAB_SPECIMEN);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (isFollowerCooking() && !follower.isRobotStuck()) return;

            incrementPathIndex();
            masterController.proceedSpecimenScoring();
        } else if (pathIndex >= 14 && pathIndex <= 23 && pathIndex % 3 == 2) {
            if (masterController.getOuttakeMechanismState() != OuttakeMechanismState.I_SPECIMEN_OUTTAKE) return;

            int scoreSpecimenIndex = (int) Math.floor((pathIndex - 14.00) / 3.00);
            followPath(1.00, scoreSpecimen.get(scoreSpecimenIndex));
        } else if (pathIndex >= 15 && pathIndex <= 21) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_SPECIMEN);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (follower.getCurrentTValue() <= 0.950) return;

            int grabSpecimenIndex = (int) Math.floor((pathIndex - 15.00) / 3.00);

            followPath(1.00, grabSpecimen.get(grabSpecimenIndex));
            masterController.proceedSpecimenScoring();
        } else if (pathIndex == 24) {
            double mappedMaxPower = map(follower.getCurrentTValue(), SCORE_SPECIMEN);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);

            if (follower.getCurrentTValue() <= 0.950) return;

            followPath(1.00, park);
            masterController.proceedSpecimenScoring();
        } else if (pathIndex == 25) {
            double mappedMaxPower = map(follower.getCurrentTValue(), PARK);
            if (!Double.isNaN(mappedMaxPower)) follower.setMaxPower(mappedMaxPower);
            if (isFollowerCooking()) return;

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
        SCORE_PRELOAD(0.65, 0.90, 1.00, 0.45),
        GRAB_PICKUP1(0.70, 0.90, 1.00, 0.30),
        GRAB_PICKUP(0.70, 0.90, 0.75, 0.30),
        GRAB_SPECIMEN(0.65, 0.85, 1.00, 0.45),
        SCORE_SPECIMEN(0.75, 0.90, 1.00, 0.45),
        PARK(0.70, 0.90, 1.00, 0.45);

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
