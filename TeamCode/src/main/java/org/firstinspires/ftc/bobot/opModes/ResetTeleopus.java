package org.firstinspires.ftc.bobot.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController;
import org.firstinspires.ftc.bobot.controllers.HorizontalSlideController.HorizontalSlideState;
import org.firstinspires.ftc.bobot.controllers.VerticalSlideController;
import org.firstinspires.ftc.bobot.controllers.VerticalSlideController.VerticalSlideState;

@TeleOp(group = "0teleopus")
public class ResetTeleopus extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        ResetState resetState = ResetState.START;
        ElapsedTime resetTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        HorizontalSlideController horizontalSlideController = new HorizontalSlideController(hardwareMap);
        VerticalSlideController verticalSlideController = new VerticalSlideController(hardwareMap);

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

            if (resetState == ResetState.START) {
                horizontalSlideController.hardResetEncoder();
                verticalSlideController.hardResetEncoder();

                resetState = ResetState.RESETTING;
                resetTimer.reset();
            } else if (resetState == ResetState.RESETTING) {
                if (resetTimer.time() >= 2000) {
                    horizontalSlideController.transitionSlide(HorizontalSlideState.EXTENDED_SHORT.position);
                    verticalSlideController.transitionSlide(VerticalSlideState.EXTENDED_MEDIUM.position);

                    resetState = ResetState.EXTENDING;
                    resetTimer.reset();
                }
            }
            else if (resetState == ResetState.EXTENDING) {
                if (!horizontalSlideController.isCooking() && !verticalSlideController.isCooking()) {
                    horizontalSlideController.transitionSlide(HorizontalSlideState.RESET.position);
                    verticalSlideController.transitionSlide(VerticalSlideState.RESET.position);

                    resetState = ResetState.RETRACTING;
                }
            } else if (resetState == ResetState.RETRACTING) {
                if (horizontalSlideController.isOverCurrent()) {
                    horizontalSlideController.transitionSlide(HorizontalSlideState.RETRACTED.position);
                    horizontalSlideController.hardResetEncoder();
                } else if (verticalSlideController.isOverCurrent()) {
                    verticalSlideController.transitionSlide(VerticalSlideState.RETRACTED.position);
                    verticalSlideController.hardResetEncoder();
                } else if (!horizontalSlideController.isCooking() && !verticalSlideController.isCooking()) {
                    resetState = ResetState.COMPLETED;
                    resetTimer.reset();
                }
            } else if (resetTimer.time() >= 2000) {
                requestOpModeStop();
            }

            horizontalSlideController.update();
            verticalSlideController.update();

            multipleTelemetry.addData("reset state", resetState);

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }

    public enum ResetState {START, RESETTING, EXTENDING, RETRACTING, COMPLETED}
}
