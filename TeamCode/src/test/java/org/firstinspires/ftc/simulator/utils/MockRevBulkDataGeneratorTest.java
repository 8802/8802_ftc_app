package org.firstinspires.ftc.simulator.utils;

import org.junit.jupiter.api.Test;
import org.openftc.revextensions2.RevBulkData;

import static org.junit.jupiter.api.Assertions.*;

class MockRevBulkDataGeneratorTest {

    @Test
    void testChangingMock() {
        MockRevBulkDataGenerator gen = new MockRevBulkDataGenerator();
        gen.encoderVals[0] = 5;

        RevBulkData data = gen.mock();
        assertEquals(5, data.getMotorCurrentPosition(0));
        assertEquals(0, data.getMotorCurrentPosition(1));
        assertEquals(0, data.getAnalogInputValue(0));
        assertEquals(false, data.getDigitalInputState(0));

        // Verify we can change the generator in real time, and that the mock responds accordingly
        gen.encoderVals[1] = 120;
        gen.digitalInputs[6] = true;

        assertEquals(120, data.getMotorCurrentPosition(1));
        assertEquals(true, data.getDigitalInputState(6));
    }
}