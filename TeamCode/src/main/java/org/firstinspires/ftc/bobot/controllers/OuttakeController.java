package org.firstinspires.ftc.bobot.controllers;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeController {

    private final Servo elbow1;
    private final Servo elbow2;
    private final Servo wrist;
    private final Servo claw;

    public OuttakeController(HardwareMap hardwareMap) {
        elbow1 = hardwareMap.servo.get("outtakeElbow1");
        elbow2 = hardwareMap.servo.get("outtakeElbow2");
        wrist = hardwareMap.servo.get("outtakeWrist");
        claw = hardwareMap.servo.get("outtakeClaw");
    }

    public void setState(OuttakeState state) {
        elbow1.setPosition(state.elbowPosition);
        elbow2.setPosition(state.elbowPosition);
        wrist.setPosition(state.wristPosition);
    }

    public void setClawState(OuttakeClawState state) {
        claw.setPosition(state.position);
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("OUTTAKE CONTROLLER");

        telemetry.addData("elbow 1 position", elbow1.getPosition());
        telemetry.addData("elbow 2 position", elbow2.getPosition());
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("claw position", claw.getPosition());
    }

    public enum OuttakeState {
        AUTONOMOUS_SAMPLE(0.800, 0.560),
        AUTONOMOUS_SPECIMEN(0.730, 0.720),
        TRANSFER(0.800, 0.560),
        SAMPLE_OUTTAKE(0.420, 0.240),
        SPECIMEN_INTAKE(0.130, 0.350),
        SPECIMEN_OUTTAKE(0.730, 0.580);

        public final double elbowPosition;
        public final double wristPosition;

        OuttakeState(double elbowPosition, double wristPosition) {
            this.elbowPosition = elbowPosition;
            this.wristPosition = wristPosition;
        }
    }

    public enum OuttakeClawState {
        CLOSED(0.570), OPENED(0.360);

        public final double position;

        OuttakeClawState(double position) {
            this.position = position;
        }
    }
}
