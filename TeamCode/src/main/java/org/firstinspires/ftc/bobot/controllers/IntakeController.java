package org.firstinspires.ftc.bobot.controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.util.Direction;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class IntakeController {

    public static double ELBOW_ROTATION_OFFSET = 0.140;

    private final NormalizedColorSensor colorSensor;

    private final Servo elbow1;
    private final Servo elbow2;
    private final Servo claw;

    public IntakeController(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        elbow1 = hardwareMap.servo.get("intakeElbow1");
        elbow2 = hardwareMap.servo.get("intakeElbow2");
        claw = hardwareMap.servo.get("intakeClaw");
    }

    public double[] getColorSensorHSV() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double hue = JavaUtil.colorToHue(colors.toColor());
        double saturation = JavaUtil.colorToSaturation(colors.toColor());
        double value = JavaUtil.colorToValue(colors.toColor());

        return new double[]{hue, saturation, value};
    }

    public SampleColor getSampleColor(double[] HSV) {
        double hue = HSV[0];
        double saturation = HSV[1];

        if (saturation <= 0.250) return SampleColor.NONE;

        if (hue <= 40 || hue >= 340) return SampleColor.RED;
        if (hue >= 50 && hue <= 100) return SampleColor.YELLOW;
        if (hue >= 140 && hue <= 235) return SampleColor.BLUE;

        return SampleColor.NONE;
    }

    public void setState(IntakeState state) {
        elbow1.setPosition(state.position);
        elbow2.setPosition(state.position);
    }

    public void rotateElbow(Direction direction) {
        int rotationDirectionMultiplier = direction == Direction.LEFT ? -1 : 1;
        double relativeOffset = rotationDirectionMultiplier * ELBOW_ROTATION_OFFSET;

        double elbow1Position = elbow1.getPosition() + relativeOffset;
        double elbow2Position = elbow2.getPosition() - relativeOffset;
        double absoluteOffset = (elbow1Position - elbow2Position) / 2;

        double elbowRotationThreshold = 2.5 * ELBOW_ROTATION_OFFSET;
        double elbowRotationCorrection = 4.0 * ELBOW_ROTATION_OFFSET;

        if (absoluteOffset >= elbowRotationThreshold) {
            elbow1Position -= elbowRotationCorrection;
            elbow2Position += elbowRotationCorrection;
        } else if (absoluteOffset <= -elbowRotationThreshold) {
            elbow1Position += elbowRotationCorrection;
            elbow2Position -= elbowRotationCorrection;
        }

        elbow1.setPosition(elbow1Position);
        elbow2.setPosition(elbow2Position);
    }

    public void lower() {
        double elbow1Position = elbow1.getPosition();
        double elbow2Position = elbow2.getPosition();

        elbow1.setPosition(elbow1Position + 0.065);
        elbow2.setPosition(elbow2Position + 0.065);
    }

    public void raise() {
        double elbow1Position = elbow1.getPosition();
        double elbow2Position = elbow2.getPosition();

        elbow1.setPosition(elbow1Position - 0.130);
        elbow2.setPosition(elbow2Position - 0.130);
    }

    public void setClawState(IntakeClawState state) {
        claw.setPosition(state.position);
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("INTAKE CONTROLLER");

        double[] HSV = getColorSensorHSV();
        telemetry.addData("hue", HSV[0]);
        telemetry.addData("saturation", HSV[1]);
        telemetry.addData("value", HSV[2]);
        telemetry.addData("sample color", getSampleColor(HSV));

        telemetry.addData("elbow 1 position", elbow1.getPosition());
        telemetry.addData("elbow 2 position", elbow2.getPosition());
        telemetry.addData("claw position", claw.getPosition());
    }

    public enum SampleColor {RED, YELLOW, BLUE, NONE}

    public enum IntakeState {
        AUTONOMOUS_SAMPLE(0.245), AUTONOMOUS_SPECIMEN(0.165), INTAKE(0.575), IDLE(0.360), TRANSFER(0.245);

        public final double position;

        IntakeState(double position) {
            this.position = position;
        }
    }

    public enum IntakeClawState {
        CLOSED(0.445), OPENED(0.225);

        public final double position;

        IntakeClawState(double position) {
            this.position = position;
        }
    }
}
