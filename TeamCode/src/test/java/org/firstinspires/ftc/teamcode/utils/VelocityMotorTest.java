
package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class VelocityMotorTest {

    @Test
    public void testMotorSpinsWhenTargetIsSet()
            throws InterruptedException {

        // Create fake hardware
        DcMotor motor = mock(DcMotor.class);
        HardwareMap hardwareMap = mock(HardwareMap.class);

        when(hardwareMap.get(DcMotor.class, "shooter"))
                .thenReturn(motor);

        when(motor.getCurrentPosition()).thenReturn(0);

        // Create controller with your current gains
        PIDFController pidf = new PIDFController(
                0.001, 0, 0, 0.000397, 0
        );

        VelocityMotor shooter = new VelocityMotor(pidf, 0.3);

        // Initialize simulated motor
        shooter.init(
                hardwareMap,
                "shooter",
                DcMotorSimple.Direction.FORWARD
        );

        // Set target velocity
        shooter.setTargetVelocity(2500);

        // Allow enough time for dt >= 0.001
        Thread.sleep(20);

        // Run velocity controller
        shooter.update();

        // Verify target was actually set
        assertEquals(
                2500,
                shooter.getTargetVelocity(),
                0.001
        );

        // Verify motor received positive power
        assertTrue(
                "Motor should receive positive power",
                shooter.getAppliedPower() > 0
        );

        // Verify the motor was commanded to run
        verify(motor, atLeastOnce()).setPower(1.0);
    }

    @Test
    public void testMotorIsInCorrectRunMode() {

        DcMotor motor = mock(DcMotor.class);
        HardwareMap hardwareMap = mock(HardwareMap.class);

        when(hardwareMap.get(DcMotor.class, "shooter"))
                .thenReturn(motor);

        PIDFController pidf = new PIDFController(
                0.001, 0, 0, 0.000397, 0
        );

        VelocityMotor shooter = new VelocityMotor(pidf, 0.3);

        shooter.init(
                hardwareMap,
                "shooter",
                DcMotorSimple.Direction.FORWARD
        );

        // Verify the encoder is reset first
        // and RUN_WITHOUT_ENCODER is set afterward.
        org.mockito.InOrder order = inOrder(motor);

        order.verify(motor).setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        order.verify(motor).setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }
}