package org.firstinspires.ftc.bobot;

import static org.firstinspires.ftc.bobot.Teleopus.DrivingMode.*;
import static org.firstinspires.ftc.bobot.controllers.MasterController.OpMode.TELEOPUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Direction;

import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState;
import org.firstinspires.ftc.bobot.controllers.MasterController;
import org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState;

import org.firstinspires.ftc.bobot.utilities.ButtonUtil.ButtonEvent;
import org.firstinspires.ftc.bobot.utilities.GamepadUtil;

import org.firstinspires.ftc.bobot.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.bobot.pedroPathing.constants.LConstants;

public class Teleopus extends LinearOpMode {

    protected Alliance alliance;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        GamepadUtil driverOp = new GamepadUtil(gamepad1);
        DrivingMode drivingMode = SCORING;

        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.startTeleopDrive();

        MasterController masterController = new MasterController(hardwareMap, alliance, TELEOPUS);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        multipleTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();

        double lastTimeStamp = 0;
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

            double period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;

            driverOp.readButtons();

            if (driverOp.wasJustPressed(GamepadKeys.Button.START))
                drivingMode = drivingMode == SCORING ? ASCENT : SCORING;

            if (drivingMode == SCORING) {
                double forwardDrive = driverOp.getLeftY(period);
                double lateralDrive = driverOp.getLeftX(period);
                double headingDrive = 0.65 * driverOp.getRightX(period);

                follower.setTeleOpMovementVectors(forwardDrive, lateralDrive, headingDrive);
                follower.update();

                ButtonEvent YButtonEvent = driverOp.getYButtonEvent();
                if (YButtonEvent == ButtonEvent.SINGLE_CLICK)
                    masterController.transitionVerticalSlide(VerticalSlideState.EXTENDED_LONG.position);
                else if (YButtonEvent == ButtonEvent.DOUBLE_CLICK)
                    masterController.transitionVerticalSlide(VerticalSlideState.EXTENDED_SHORT.position);
                else if (YButtonEvent == ButtonEvent.HOLDING)
                    masterController.transitionVerticalSlide(VerticalSlideState.RESET.position);

                ButtonEvent BButtonEvent = driverOp.getBButtonEvent();
                if (BButtonEvent == ButtonEvent.SINGLE_CLICK)
                    masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_LONG.position);
                else if (BButtonEvent == ButtonEvent.DOUBLE_CLICK)
                    masterController.transitionHorizontalSlide(HorizontalSlideState.EXTENDED_SHORT.position);
                else if (BButtonEvent == ButtonEvent.HOLDING)
                    masterController.transitionHorizontalSlide(HorizontalSlideState.RESET.position);

                if (driverOp.wasJustPressed(GamepadKeys.Button.A)) masterController.proceedSampleScoring();
                if (driverOp.wasJustPressed(GamepadKeys.Button.X)) masterController.proceedSpecimenScoring();

                if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                    masterController.rotateElbow(Direction.LEFT);
                else if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                    masterController.rotateElbow(Direction.RIGHT);

                if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
                    masterController.transitionVerticalSlide(Direction.UP, period);
                else if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                    masterController.transitionVerticalSlide(Direction.DOWN, period);

                if (driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                    masterController.transitionHorizontalSlide(Direction.FORWARD, period);
                else if (driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                    masterController.transitionHorizontalSlide(Direction.BACKWARDS, period);
            } else {
                if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER))
                    masterController.transitionVerticalSlide(Direction.UP, 5 * period);
                else if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                    masterController.transitionVerticalSlide(Direction.DOWN, 5 * period);
            }

            masterController.update();
            masterController.debug(multipleTelemetry);

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }

    public enum Alliance {BLUE, RED}

    public enum DrivingMode {SCORING, ASCENT}
}
