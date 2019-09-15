package org.firstinspires.ftc.simulator;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.math.Pose;

import java.io.IOException;
import java.lang.reflect.Type;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

public class RXThread extends Thread {
    private final static int PACKETSIZE = 1000;
    private final static int JAVA_SERVER_PORT = 4445;
    protected DatagramSocket rxsocket;
    private boolean terminate;

    volatile Gamepad gp1, gp2;
    Gson gson;

    public RXThread(Gamepad gp1, Gamepad gp2) {
        super();
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    @Override
    public void start() {
        try {
            rxsocket = new DatagramSocket(JAVA_SERVER_PORT);
            rxsocket.setSoTimeout(250);
        } catch (SocketException e) {
            e.printStackTrace();
        }
        this.gson = new Gson();
        this.terminate = false;
        super.start();
    }

    @Override
    public void run() {
        while(!terminate) {
            byte buf[] = new byte[PACKETSIZE];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            try {
                rxsocket.receive(packet);
                String data = new String(packet.getData(), 0, packet.getLength());
                Gamepad sentGamepad = gson.fromJson(data, Gamepad.class);
                gp1.left_stick_y = sentGamepad.left_stick_y;
                gp1.left_stick_x = sentGamepad.left_stick_x;
                gp1.right_stick_x = sentGamepad.right_stick_x;
                gp1.right_stick_y = sentGamepad.right_stick_y;
                gp1.left_stick_button = sentGamepad.left_stick_button;
                gp1.right_stick_button = sentGamepad.right_stick_button;
            } catch(IOException e) {
            }
        }
        rxsocket.close();
    }

    public void kill() {
        this.terminate = true;
    }
}
