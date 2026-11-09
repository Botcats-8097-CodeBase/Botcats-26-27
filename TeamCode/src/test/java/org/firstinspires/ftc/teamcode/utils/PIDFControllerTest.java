
package org.firstinspires.ftc.teamcode.utils;

import org.junit.Test;
import static org.junit.Assert.*;

public class PIDFControllerTest {

    // Test 1: Feedforward
    @Test
    public void testFeedforward() {
        PIDFController pidf = new PIDFController(
                0, 0, 0, 0.000397, 0
        );

        pidf.setSetpoint(2500);

        double output = pidf.update(0, 0.02);

        // 0.000397 * 2500 = 0.9925
        assertEquals(0.9925, output, 0.000001);
    }

    // Test 2: Proportional control
    @Test
    public void testProportional() {
        PIDFController pidf = new PIDFController(
                0.001, 0, 0, 0, 0
        );

        pidf.setSetpoint(2500);

        double output = pidf.update(2000, 0.02);

        // Error = 500
        // P output = 0.001 * 500 = 0.5
        assertEquals(0.5, output, 0.000001);
    }

    // Test 3: Integral control
    @Test
    public void testIntegral() {
        PIDFController pidf = new PIDFController(
                0, 0.01, 0, 0, 1000
        );

        pidf.setSetpoint(100);

        double output = 0;

        for (int i = 0; i < 50; i++) {
            output = pidf.update(0, 0.02);
        }

        // Integral = 100 * 1 second = 100
        // I output = 0.01 * 100 = 1.0
        assertEquals(1.0, output, 0.000001);
    }

    // Test 4: Derivative control
    @Test
    public void testDerivative() {
        PIDFController pidf = new PIDFController(
                0, 0, 0.001, 0, 0
        );

        pidf.setSetpoint(1000);

        // Initialize previous error
        pidf.update(0, 0.02);

        // Error changes from 1000 to 800
        double output = pidf.update(200, 0.02);

        // D = 0.001 * (-200 / 0.02) = -10
        assertEquals(-10.0, output, 0.000001);
    }

    // Test 5: Combined PIDF
    @Test
    public void testCombinedPIDF() {
        PIDFController pidf = new PIDFController(
                0.001, 0, 0, 0.000397, 0
        );

        pidf.setSetpoint(2500);

        double output = pidf.update(2000, 0.02);

        // P = 0.5
        // F = 0.9925
        // Total = 1.4925 (before motor clamping)
        assertEquals(1.4925, output, 0.000001);
    }

    // Test 6: Reset integral history
    @Test
    public void testReset() {
        PIDFController pidf = new PIDFController(
                0, 1, 0, 0, 1000
        );

        pidf.setSetpoint(100);

        pidf.update(0, 0.02);
        pidf.reset();

        // After reset, error is zero,
        // so the integral should remain zero.
        double output = pidf.update(100, 0.02);

        assertEquals(0.0, output, 0.000001);
    }

    // Test 7: Zero error with feedforward
    @Test
    public void testZeroError() {
        PIDFController pidf = new PIDFController(
                0.001, 0, 0, 0.000397, 0
        );

        pidf.setSetpoint(2500);

        double output = pidf.update(2500, 0.02);

        assertEquals(0.9925, output, 0.000001);
    }
}