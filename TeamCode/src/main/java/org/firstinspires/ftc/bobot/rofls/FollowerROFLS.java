package org.firstinspires.ftc.bobot.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.bobot.utilities.GamepadUtil;

import org.firstinspires.ftc.bobot.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.bobot.pedroPathing.constants.LConstants;

@TeleOp(group = "1ROFLS")
public class FollowerROFLS extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        GamepadUtil driverOp = new GamepadUtil(gamepad1);

        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.startTeleopDrive();

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

            double forwardDrive = driverOp.getLeftY(period);
            double lateralDrive = driverOp.getLeftX(period);
            double headingDrive = 0.65 * driverOp.getRightX(period);

            multipleTelemetry.addData("forward drive", forwardDrive);
            multipleTelemetry.addData("lateral drive", lateralDrive);
            multipleTelemetry.addData("heading drive", headingDrive);

            follower.setTeleOpMovementVectors(forwardDrive, lateralDrive, headingDrive);
            follower.update();

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }
}
