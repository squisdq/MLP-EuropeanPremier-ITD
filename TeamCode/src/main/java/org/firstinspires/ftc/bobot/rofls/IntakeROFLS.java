package org.firstinspires.ftc.bobot.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.bobot.controllers.IntakeController;
import org.firstinspires.ftc.bobot.controllers.IntakeController.IntakeClawState;
import org.firstinspires.ftc.bobot.controllers.IntakeController.IntakeState;

@Config
@TeleOp(group = "1ROFLS")
public class IntakeROFLS extends LinearOpMode {

    public static double elbow1Position = IntakeState.INTAKE.position;
    public static double elbow2Position = IntakeState.INTAKE.position;
    public static double clawPosition = IntakeClawState.OPENED.position;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Servo elbow1 = hardwareMap.servo.get("intakeElbow1");
        Servo elbow2 = hardwareMap.servo.get("intakeElbow2");
        Servo claw = hardwareMap.servo.get("intakeClaw");

        IntakeController intakeController = new IntakeController(hardwareMap);

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

            elbow1.setPosition(elbow1Position);
            elbow2.setPosition(elbow2Position);
            claw.setPosition(clawPosition);

            intakeController.debug(multipleTelemetry);

            double frequency = 1 / period;
            double voltage = voltageSensor.getVoltage();

            multipleTelemetry.addData("frequency", frequency);
            multipleTelemetry.addData("voltage", voltage);
            multipleTelemetry.update();
        }
    }
}
