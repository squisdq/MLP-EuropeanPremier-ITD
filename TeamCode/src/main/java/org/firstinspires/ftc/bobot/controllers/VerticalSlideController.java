package org.firstinspires.ftc.bobot.controllers;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import static org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState.EXTENDED_LONG;
import static org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState.RETRACTED;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.util.Direction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class VerticalSlideController {

    public static int EXTENSION_RATE = 2000;

    public static double KP = 0; // 0.0175;
    public static double KD = 0; // 0.0004;

    private final PDController controller;

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private int positionOffset = 0;
    private int targetPosition = 0;

    private boolean controllerActive = true;

    public VerticalSlideController(HardwareMap hardwareMap) {
        controller = new PDController(KP, KD);
        controller.setSetPoint(targetPosition);
        controller.setTolerance(50, 50);

        motor1 = hardwareMap.get(DcMotorEx.class, "verticalSlide1");
        motor2 = hardwareMap.get(DcMotorEx.class, "verticalSlide2");

        motor1.setCurrentAlert(5000, MILLIAMPS);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isCooking() {
        return !controller.atSetPoint();
    }

    public boolean isOverCurrent() {
        return motor1.isOverCurrent();
    }

    public int getCurrentPosition() {
        return motor1.getCurrentPosition() - positionOffset;
    }

    public void resetEncoder() {
        positionOffset += getCurrentPosition();
    }

    public void hardResetEncoder() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void transitionSlide(int position) {
        targetPosition = position;
        controller.setSetPoint(targetPosition);
    }

    public void transitionSlide(Direction direction, double period) {
        int multiplier = direction == Direction.UP ? 1 : -1;

        int currentPosition = getCurrentPosition();
        int offset = (int) (multiplier * EXTENSION_RATE * period);

        int minExtension = RETRACTED.position;
        int maxExtension = EXTENDED_LONG.position;

        targetPosition = Range.clip(currentPosition + offset, minExtension, maxExtension);
        controller.setSetPoint(targetPosition);
    }

    public void setControllerActive(boolean active) {
        controllerActive = active;
    }

    public void update() {
        controller.setPID(KP, 0, KD);

        int currentPosition = getCurrentPosition();

        double rawPower = controllerActive ? controller.calculate(currentPosition) : 0;
        double power = Math.abs(rawPower) >= 0.05 ? Range.clip(rawPower, -1, 1) : 0;

        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void debug(MultipleTelemetry telemetry) {
        telemetry.addLine("VERTICAL SLIDE CONTROLLER");

        telemetry.addData("is cooking", isCooking());
        telemetry.addData("is over current", isOverCurrent());

        telemetry.addData("current position", getCurrentPosition());

        telemetry.addData("target position", targetPosition);

        telemetry.addData("motor current", motor1.getCurrent(MILLIAMPS));
        telemetry.addData("motor power", motor1.getPower());
    }

    public enum VerticalSlideState {
        RESET(-50), RETRACTED(0), EXTENDED_SHORT(1000), EXTENDED_MEDIUM(1300), EXTENDED_LONG(2300);

        public final int position;

        VerticalSlideState(int position) {
            this.position = position;
        }
    }
}
