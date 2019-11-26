package org.firstinspires.ftc.simulator.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.mockito.Mockito;
import org.openftc.revextensions2.RevBulkData;

public class MockRevBulkDataGenerator {


    public int[] encoderVals;
    public int[] analogInputs;
    public boolean[] digitalInputs;

    public MockRevBulkDataGenerator() {
        encoderVals = new int[] {0, 0, 0, 0};
        analogInputs = new int[] {0, 0, 0, 0};
        digitalInputs = new boolean[]{false, false, false, false, false, false, false, false};
    }

    public RevBulkData mock() {
        RevBulkData data = Mockito.mock(RevBulkData.class);
        Mockito.when(data.getMotorCurrentPosition(Mockito.anyInt()))
                .thenAnswer(invocation ->
                        encoderVals[invocation.getArgumentAt(0, Integer.class)]);
        Mockito.when(data.getAnalogInputValue(Mockito.anyInt()))
                .thenAnswer(invocation ->
                        analogInputs[invocation.getArgumentAt(0, Integer.class)]);
        Mockito.when(data.getDigitalInputState(Mockito.anyInt()))
                .thenAnswer(invocation ->
                        digitalInputs[invocation.getArgumentAt(0, Integer.class)]);
        return data;
    }
}
