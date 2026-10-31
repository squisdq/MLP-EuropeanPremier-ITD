package org.firstinspires.ftc.bobot.controllers;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import static org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState.EXTENDED_LONG;
import static org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState.RETRACTED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.util.Direction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class HorizontalSlideController {

    public static int EXTENSION_RATE = 1000;

    public static double KP = 0.0175;
    public static double KD = 0.0004;

    private final PDController controller;

    private final DcMotorEx motor;

    private int encoderOffset = 0;
    private int targetPosition = 0;

    public HorizontalSlideController(HardwareMap hardwareMap) {
        controller = new PDController(KP, KD);
        controller.setSetPoint(targetPosition);
        controller.setTolerance(50, 50);

        motor = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        motor.setCurrentAlert(7000, MILLIAMPS);
    }

    public boolean isCooking() {
        return !controller.atSetPoint();
    }

    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition() - encoderOffset;
    }

    public void resetEncoder() {
        encoderOffset += getCurrentPosition();
    }

    public void hardResetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void transitionSlide(int position) {
        targetPosition = position;
        controller.setSetPoint(targetPosition);
    }

    public void transitionSlide(Direction direction, double period) {
        int multiplier = direction == Direction.FORWARD ? 1 : -1;

        int currentPosition = getCurrentPosition();
        int offset = (int) (multiplier * EXTENSION_RATE * period);

        int minExtension = RETRACTED.position;
        int maxExtension = EXTENDED_LONG.position;

        targetPosition = Range.clip(currentPosition + offset, minExtension, maxExtension);
        controller.setSetPoint(targetPosition);
    }

    public void update() {
        controller.setPID(KP, 0, KD);

        int currentPosition = getCurrentPosition();

        double rawPower = controller.calculate(currentPosition);
        double power = Math.abs(rawPower) >= 0.05 ? Range.clip(rawPower, -1, 1) : 0;

        motor.setPower(power);
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("HORIZONTAL SLIDE CONTROLLER");

        telemetry.addData("is cooking", isCooking());
        telemetry.addData("is over current", isOverCurrent());

        telemetry.addData("current position", getCurrentPosition());

        telemetry.addData("target position", targetPosition);

        telemetry.addData("motor current", motor.getCurrent(MILLIAMPS));
        telemetry.addData("motor power", motor.getPower());
    }

    public enum HorizontalSlideState {
        TRANSFER(-200), RESET(-25), RETRACTED(0), EXTENDED_SHORT(200), EXTENDED_MEDIUM(400), EXTENDED_LONG(600);

        public final int position;

        HorizontalSlideState(int position) {
            this.position = position;
        }
    }
}
