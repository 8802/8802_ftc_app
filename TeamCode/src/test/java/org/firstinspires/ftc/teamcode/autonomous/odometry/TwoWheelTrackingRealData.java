package org.firstinspires.ftc.teamcode.autonomous.odometry;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import org.openftc.revextensions2.RevBulkData;

import java.io.IOException;
import java.io.InputStream;
import java.util.Scanner;

class TwoWheelTrackingRealData {

    private TwoWheelTrackingLocalizer getNewLocalizer() {
        return new TwoWheelTrackingLocalizer(0, 1);
    }

    // Lateral encoder is on port 2 instead of port 1 because port 1 is
    // reserved for the second parallel encoder, which we just aren't using here
    private RevBulkData genFakeData(int parallelEncoder, int lateralEncoder) {
        int[] encoderVals = new int[4];
        encoderVals[TwoWheelTrackingLocalizer.PARALLEL_ENCODER_PORT] = parallelEncoder;
        encoderVals[TwoWheelTrackingLocalizer.LATERAL_ENCODER_PORT] = lateralEncoder;

        RevBulkData data = Mockito.mock(RevBulkData.class);
        Mockito.when(data.getMotorCurrentPosition(Mockito.anyInt()))
                .thenAnswer(invocation ->
                        encoderVals[invocation.getArgumentAt(0, Integer.class)]);
        return data;
    }

    double THRESHOLD = 0.1;

    private boolean roughApproxEquals(Pose p1, Pose p2) {
        return Math.abs(p1.x - p2.x) < THRESHOLD &&
                Math.abs(p1.y - p2.y) < THRESHOLD &&
                Math.abs(p1.heading - p2.heading) < THRESHOLD;
    }

    @Test
    void testTrackingWheelUpdate() throws IOException {
        // Move forward 24 inches
        TwoWheelTrackingLocalizer.LATERAL_X_POS = -12;
        TwoWheelTrackingLocalizer.PARALLEL_Y_POS = -12;

        InputStream inputStream = this.getClass().getClassLoader().getResourceAsStream("spinvalues.json");
        Scanner sc = new Scanner(inputStream);
        StringBuffer sb = new StringBuffer();
        while(sc.hasNext()){
            sb.append(sc.nextLine());
        }
        Gson gson = new GsonBuilder().create();
        double[][] d = gson.fromJson(sb.toString(), double[][].class);

        for (int i = 0; i < 240; i++) {
            TwoWheelTrackingLocalizer.LATERAL_X_POS += 0.1;
            for (int k = 0; k < 240; k++) {
                TwoWheelTrackingLocalizer.PARALLEL_Y_POS += 0.1;
                TwoWheelTrackingLocalizer localizer = getNewLocalizer();
                for (double[] delta : d) {
                    localizer.updateFromRelative(delta);
                }
                if (localizer.currentPosition.radius() < 4) {
                    System.out.println(TwoWheelTrackingLocalizer.LATERAL_X_POS);
                    System.out.println(TwoWheelTrackingLocalizer.PARALLEL_Y_POS);
                    System.out.println(localizer.currentPosition.toString());
                }
            }
            System.out.println(i + "/240");
            TwoWheelTrackingLocalizer.PARALLEL_Y_POS = -12;
        }
    }
}