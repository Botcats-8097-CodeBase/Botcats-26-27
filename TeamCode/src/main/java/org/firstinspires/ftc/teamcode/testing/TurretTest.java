package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.LowPassFilter;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.List;

@TeleOp (name = "turretTest")
public class TurretTest extends LinearOpMode {

    List<LynxModule> allHubs;

    ElapsedTime elapsedTime = new ElapsedTime();
    LowPassFilter velocityFilter = new LowPassFilter(0.3);
    DcMotor spinner;

    PIDController velocityPID = new PIDController(0.001, 0, 0, 0);
    double targVel = 0;
    double lastPos = 0;

    double kV = 0.000397;

    JoinedTelemetry pTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

    @Override
    public void runOpMode() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        spinner = hardwareMap.get(DcMotor.class, "pollenSpinner");
        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        elapsedTime.reset();
        lastPos = spinner.getCurrentPosition();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double dt = elapsedTime.seconds();
            elapsedTime.reset();

            if (dt <= 0) continue;

            double currPos = spinner.getCurrentPosition();

            // enc. vel
            double rawVel = (currPos - lastPos) / dt;
            lastPos = currPos;
            velocityFilter.update(rawVel);

            double actlVel = velocityFilter.getValue();

            // vel PID
            double errorVel = targVel - actlVel;
            double correction = velocityPID.update(errorVel, dt);

            // feedforward + PID
            double power = kV * targVel + correction;
            power = Math.max(-1, Math.min(1, power));

            spinner.setPower(power);

            if (gamepad1.dpadUpWasPressed()) {
                targVel += 100;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targVel -= 100;
            }
            if (gamepad1.dpadRightWasPressed()) {
                targVel += 10.0;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                targVel -= 10.0;
            }

            pTelemetry.addData("targVel", targVel);
            pTelemetry.addData("actlVel", actlVel);
            pTelemetry.addData("velError", errorVel);
            pTelemetry.addData("spinnerVel", power);
            pTelemetry.update();
        }
    }
}
