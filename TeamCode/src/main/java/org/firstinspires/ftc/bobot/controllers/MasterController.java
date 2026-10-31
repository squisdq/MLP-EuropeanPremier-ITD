package org.firstinspires.ftc.bobot.controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.util.Direction;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.bobot.Teleopus.Alliance;

import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState;
import org.firstinspires.ftc.bobot.controllers.IntakeController.IntakeClawState;
import org.firstinspires.ftc.bobot.controllers.IntakeController.IntakeState;
import org.firstinspires.ftc.bobot.controllers.IntakeController.SampleColor;
import org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState;
import org.firstinspires.ftc.bobot.controllers.OuttakeController.OuttakeClawState;
import org.firstinspires.ftc.bobot.controllers.OuttakeController.OuttakeState;

public class MasterController {

    private final HorizontalSlideController horizontalSlideController;
    private final IntakeController intakeController;
    private final VerticalSlideController verticalSlideController;
    private final OuttakeController outtakeController;

    private final ElapsedTime transitionTimer;
    private final SampleColor allianceSampleColor;

    private IntakeMechanismState intakeMechanismState;
    private OuttakeMechanismState outtakeMechanismState;

    public MasterController(HardwareMap hardwareMap, Alliance alliance, OpMode opMode) {
        horizontalSlideController = new HorizontalSlideController(hardwareMap);
        intakeController = new IntakeController(hardwareMap);
        verticalSlideController = new VerticalSlideController(hardwareMap);
        outtakeController = new OuttakeController(hardwareMap);

        transitionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        allianceSampleColor = alliance == Alliance.BLUE ? SampleColor.BLUE : SampleColor.RED;

        if (opMode == OpMode.AUTONOMOUS_SAMPLE) {
            horizontalSlideController.transitionSlide(HorizontalSlideState.RESET.position);
            intakeController.setState(IntakeState.AUTONOMOUS_SAMPLE);
            intakeController.setClawState(IntakeClawState.OPENED);
            verticalSlideController.transitionSlide(VerticalSlideState.RESET.position);
            outtakeController.setState(OuttakeState.AUTONOMOUS_SAMPLE);
            outtakeController.setClawState(OuttakeClawState.CLOSED);

            intakeMechanismState = IntakeMechanismState.A_AUTONOMOUS;
            outtakeMechanismState = OuttakeMechanismState.A_AUTONOMOUS_SAMPLE;
        } else if (opMode == OpMode.AUTONOMOUS_SPECIMEN) {
            horizontalSlideController.transitionSlide(HorizontalSlideState.RESET.position);
            intakeController.setState(IntakeState.AUTONOMOUS_SPECIMEN);
            intakeController.setClawState(IntakeClawState.OPENED);
            verticalSlideController.transitionSlide(VerticalSlideState.RESET.position);
            outtakeController.setState(OuttakeState.AUTONOMOUS_SPECIMEN);
            outtakeController.setClawState(OuttakeClawState.CLOSED);

            intakeMechanismState = IntakeMechanismState.A_AUTONOMOUS;
            outtakeMechanismState = OuttakeMechanismState.A_AUTONOMOUS_SPECIMEN;
        } else if (opMode == OpMode.TELEOPUS) {
            horizontalSlideController.transitionSlide(HorizontalSlideState.RETRACTED.position);
            intakeController.setState(IntakeState.INTAKE);
            intakeController.setClawState(IntakeClawState.OPENED);
            verticalSlideController.transitionSlide(VerticalSlideState.RETRACTED.position);
            outtakeController.setState(OuttakeState.TRANSFER);
            outtakeController.setClawState(OuttakeClawState.OPENED);

            intakeMechanismState = IntakeMechanismState.I_INTAKE;
            outtakeMechanismState = OuttakeMechanismState.I_TRANSFER;
        }
    }

    public boolean isHorizontalSlideCooking() {
        return horizontalSlideController.isCooking();
    }

    public void transitionHorizontalSlide(int position) {
        horizontalSlideController.transitionSlide(position);
    }

    public void transitionHorizontalSlide(Direction direction, double period) {
        horizontalSlideController.transitionSlide(direction, period);
    }

    public boolean isVerticalSlideCooking() {
        return verticalSlideController.isCooking();
    }

    public void transitionVerticalSlide(int position) {
        verticalSlideController.transitionSlide(position);
    }

    public void transitionVerticalSlide(Direction direction, double period) {
        verticalSlideController.transitionSlide(direction, period);
    }

    public void rotateElbow(Direction direction) {
        intakeController.rotateElbow(direction);
    }

    public void proceedSampleScoring() {
        if (outtakeMechanismState == OuttakeMechanismState.I_TRANSFER)
            intakeMechanismState = IntakeMechanismState.A_INTAKE1;
        else if (outtakeMechanismState == OuttakeMechanismState.A_SAMPLE_OUTTAKE1)
            intakeMechanismState = IntakeMechanismState.A_INTAKE1;
        else if (outtakeMechanismState == OuttakeMechanismState.I_SPECIMEN_INTAKE)
            intakeMechanismState = IntakeMechanismState.A_INTAKE1;
        else if (outtakeMechanismState == OuttakeMechanismState.I_SAMPLE_OUTTAKE)
            outtakeMechanismState = OuttakeMechanismState.A_SAMPLE_OUTTAKE1;
    }

    public void proceedSpecimenScoring() {
        if (outtakeMechanismState == OuttakeMechanismState.I_TRANSFER)
            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_INTAKE0;
        else if (outtakeMechanismState == OuttakeMechanismState.I_SAMPLE_OUTTAKE)
            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_INTAKE0;
        else if (outtakeMechanismState == OuttakeMechanismState.I_SPECIMEN_INTAKE)
            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_INTAKE1;
        else if (outtakeMechanismState == OuttakeMechanismState.I_SPECIMEN_OUTTAKE)
            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_OUTTAKE1;
    }

    private boolean isIntakeMechanismActive() {
        return intakeMechanismState.toString().charAt(0) == 'A';
    }

    public void setVerticalSlideControllerActive(boolean active) {
        verticalSlideController.setControllerActive(active);
    }

    private void updateIntakeMechanism() {
        if (intakeMechanismState == IntakeMechanismState.A_AUTONOMOUS) {
            intakeController.setState(IntakeState.INTAKE);

            intakeMechanismState = IntakeMechanismState.I_INTAKE;
        } else if (intakeMechanismState == IntakeMechanismState.A_INTAKE1) {
            intakeController.lower();
            intakeController.setClawState(IntakeClawState.CLOSED);

            intakeMechanismState = IntakeMechanismState.A_INTAKE2;
            transitionTimer.reset();
        } else if (intakeMechanismState == IntakeMechanismState.A_INTAKE2) {
            if (transitionTimer.time() <= 250) return;

            intakeController.raise();

            intakeMechanismState = IntakeMechanismState.A_INTAKE3;
        } else if (intakeMechanismState == IntakeMechanismState.A_INTAKE3) {
            if (transitionTimer.time() <= 500) return;

            double[] HSV = intakeController.getColorSensorHSV();
            SampleColor sampleColor = intakeController.getSampleColor(HSV);
            if (sampleColor == allianceSampleColor || sampleColor == SampleColor.YELLOW) {
                horizontalSlideController.transitionSlide(HorizontalSlideState.RESET.position);
                intakeController.setState(IntakeState.IDLE);

                intakeMechanismState = IntakeMechanismState.A_TRANSFER1;
                outtakeMechanismState = OuttakeMechanismState.A_TRANSFER1;
                transitionTimer.reset();
            } else {
                intakeController.lower();
                intakeController.setClawState(IntakeClawState.OPENED);

                intakeMechanismState = IntakeMechanismState.A_INTAKE_FAILED;
                transitionTimer.reset();
            }
        } else if (intakeMechanismState == IntakeMechanismState.A_TRANSFER1) {
            if (isHorizontalSlideCooking() || isVerticalSlideCooking() || transitionTimer.time() <= 500) return;

            horizontalSlideController.transitionSlide(HorizontalSlideState.TRANSFER.position);
            intakeController.setState(IntakeState.TRANSFER);
            setVerticalSlideControllerActive(false);

            intakeMechanismState = IntakeMechanismState.A_TRANSFER2;
            transitionTimer.reset();
        } else if (intakeMechanismState == IntakeMechanismState.A_TRANSFER2) {
            if (transitionTimer.time() <= 300) return;

            intakeController.setClawState(IntakeClawState.OPENED);

            intakeMechanismState = IntakeMechanismState.A_TRANSFER3;
        } else if (intakeMechanismState == IntakeMechanismState.A_TRANSFER3) {
            if (transitionTimer.time() <= 600) return;

            horizontalSlideController.transitionSlide(HorizontalSlideState.RETRACTED.position);
            intakeController.setState(IntakeState.INTAKE);

            intakeMechanismState = IntakeMechanismState.I_INTAKE;
        } else if (intakeMechanismState == IntakeMechanismState.A_INTAKE_FAILED) {
            if (transitionTimer.time() <= 250) return;

            intakeMechanismState = IntakeMechanismState.I_INTAKE;
        }
    }

    private boolean isOuttakeMechanismActive() {
        return outtakeMechanismState.toString().charAt(0) == 'A';
    }

    private void updateOuttakeMechanism() {
        if (outtakeMechanismState == OuttakeMechanismState.A_AUTONOMOUS_SAMPLE) {
            verticalSlideController.transitionSlide(VerticalSlideState.EXTENDED_LONG.position);
            outtakeController.setState(OuttakeState.SAMPLE_OUTTAKE);

            outtakeMechanismState = OuttakeMechanismState.I_SAMPLE_OUTTAKE;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_AUTONOMOUS_SPECIMEN) {
            verticalSlideController.transitionSlide(VerticalSlideState.EXTENDED_MEDIUM.position);
            outtakeController.setState(OuttakeState.SPECIMEN_OUTTAKE);

            outtakeMechanismState = OuttakeMechanismState.I_SPECIMEN_OUTTAKE;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_TRANSFER1) {
            verticalSlideController.transitionSlide(VerticalSlideState.RETRACTED.position);
            outtakeController.setState(OuttakeState.TRANSFER);

            outtakeMechanismState = OuttakeMechanismState.A_TRANSFER2;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_TRANSFER2) {
            if (intakeMechanismState == IntakeMechanismState.A_TRANSFER1 || transitionTimer.time() <= 200) return;

            outtakeController.setClawState(OuttakeClawState.CLOSED);

            outtakeMechanismState = OuttakeMechanismState.A_TRANSFER3;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_TRANSFER3) {
            if (transitionTimer.time() <= 400) return;

            setVerticalSlideControllerActive(true);
            outtakeController.setState(OuttakeState.SAMPLE_OUTTAKE);

            outtakeMechanismState = OuttakeMechanismState.I_SAMPLE_OUTTAKE;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SAMPLE_OUTTAKE1) {
            outtakeController.setClawState(OuttakeClawState.OPENED);

            outtakeMechanismState = OuttakeMechanismState.A_SAMPLE_OUTTAKE2;
            transitionTimer.reset();
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SAMPLE_OUTTAKE2) {
            if (transitionTimer.time() <= 200) return;

            outtakeController.setState(OuttakeState.TRANSFER);

            outtakeMechanismState = OuttakeMechanismState.A_SAMPLE_OUTTAKE3;
            transitionTimer.reset();
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SAMPLE_OUTTAKE3) {
            int targetPosition = verticalSlideController.getTargetPosition();
            int targetPositionThreshold = VerticalSlideState.EXTENDED_SHORT.position;
            if (targetPosition > targetPositionThreshold && transitionTimer.time() <= 500) return;

            verticalSlideController.transitionSlide(VerticalSlideState.RESET.position);

            outtakeMechanismState = OuttakeMechanismState.I_TRANSFER;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SPECIMEN_INTAKE0) {
            outtakeController.setState(OuttakeState.SPECIMEN_INTAKE);
            outtakeController.setClawState(OuttakeClawState.OPENED);

            outtakeMechanismState = OuttakeMechanismState.I_SPECIMEN_INTAKE;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SPECIMEN_INTAKE1) {
            outtakeController.setClawState(OuttakeClawState.CLOSED);

            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_INTAKE2;
            transitionTimer.reset();
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SPECIMEN_INTAKE2) {
            if (transitionTimer.time() <= 200) return;

            verticalSlideController.transitionSlide(VerticalSlideState.EXTENDED_MEDIUM.position);
            outtakeController.setState(OuttakeState.SPECIMEN_OUTTAKE);

            outtakeMechanismState = OuttakeMechanismState.I_SPECIMEN_OUTTAKE;
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SPECIMEN_OUTTAKE1) {
            outtakeController.setClawState(OuttakeClawState.OPENED);

            outtakeMechanismState = OuttakeMechanismState.A_SPECIMEN_OUTTAKE2;
            transitionTimer.reset();
        } else if (outtakeMechanismState == OuttakeMechanismState.A_SPECIMEN_OUTTAKE2) {
            if (transitionTimer.time() <= 200) return;

            verticalSlideController.transitionSlide(VerticalSlideState.RESET.position);
            outtakeController.setState(OuttakeState.SPECIMEN_INTAKE);

            outtakeMechanismState = OuttakeMechanismState.I_SPECIMEN_INTAKE;
        }
    }

    public void update() {
        horizontalSlideController.update();
        verticalSlideController.update();

        if (isIntakeMechanismActive()) updateIntakeMechanism();
        if (isOuttakeMechanismActive()) updateOuttakeMechanism();

        if (horizontalSlideController.isOverCurrent()) {
            int targetPosition = horizontalSlideController.getTargetPosition();
            if (targetPosition == HorizontalSlideState.RESET.position) {
                horizontalSlideController.resetEncoder();
                horizontalSlideController.transitionSlide(HorizontalSlideState.RETRACTED.position);
            }
        } else if (verticalSlideController.isOverCurrent()) {
            int targetPosition = verticalSlideController.getTargetPosition();
            if (targetPosition == VerticalSlideState.RESET.position) {
                verticalSlideController.resetEncoder();
                verticalSlideController.transitionSlide(VerticalSlideState.RETRACTED.position);
            }
        }
    }

    public IntakeMechanismState getIntakeMechanismState() {
        return intakeMechanismState;
    }

    public OuttakeMechanismState getOuttakeMechanismState() {
        return outtakeMechanismState;
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("MASTER CONTROLLER");

        telemetry.addData("intake mechanism state", intakeMechanismState);
        telemetry.addData("outtake mechanism state", outtakeMechanismState);
    }

    public enum IntakeMechanismState {
        A_AUTONOMOUS,
        I_INTAKE, A_INTAKE1, A_INTAKE2, A_INTAKE3, A_INTAKE_FAILED,
        A_TRANSFER1, A_TRANSFER2, A_TRANSFER3,
    }

    public enum OuttakeMechanismState {
        A_AUTONOMOUS_SAMPLE,
        A_AUTONOMOUS_SPECIMEN,
        I_TRANSFER, A_TRANSFER1, A_TRANSFER2, A_TRANSFER3,
        I_SAMPLE_OUTTAKE, A_SAMPLE_OUTTAKE1, A_SAMPLE_OUTTAKE2, A_SAMPLE_OUTTAKE3,
        I_SPECIMEN_INTAKE, A_SPECIMEN_INTAKE0, A_SPECIMEN_INTAKE1, A_SPECIMEN_INTAKE2,
        I_SPECIMEN_OUTTAKE, A_SPECIMEN_OUTTAKE1, A_SPECIMEN_OUTTAKE2,
    }

    public enum OpMode {AUTONOMOUS_SAMPLE, AUTONOMOUS_SPECIMEN, TELEOPUS}
}
