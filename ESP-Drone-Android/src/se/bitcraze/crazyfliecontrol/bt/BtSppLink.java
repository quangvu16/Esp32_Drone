/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * Crazyflie Nano Quadcopter Client
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

package se.bitcraze.crazyfliecontrol.bt;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.os.Build;

import se.bitcraze.crazyflie.lib.crtp.CrtpDriver;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;

/**
 * Bluetooth Classic SPP transport (PROTOCOL.md section 8, link profile 2).
 *
 * <p>Wire bytes are identical to UDP — {@code [CRTP ...][uint8 checksum]} — but SPP is a
 * byte stream rather than a datagram service, so inbound data needs framing. The firmware
 * writes one packet per {@code BluetoothSerial.write}, and the checksum (low byte of the sum
 * of every CRTP byte including the header) is strong enough to resynchronise on, so the
 * receiver scans for a length whose checksum validates.
 *
 * <p>Note the radio is shared with WiFi on the ESP32: only one of the two can be up at a time.
 */
@SuppressLint("NewApi")
public class BtSppLink extends CrtpDriver {

    private final Logger mLogger = LoggerFactory.getLogger("BtSppLink");

    /** Standard Serial Port Profile UUID — the firmware's BluetoothSerial uses the same. */
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    /** Matches the firmware's WiFi AP SSID and the BLE advertised name. */
    private static final String DRONE_DEVICE_NAME = "ESP-Drone";

    /** CRTP_MAX_SIZE (30) + header already included + 1 checksum byte. */
    private static final int MAX_FRAME = 32;

    private final Context mContext;
    private final BlockingQueue<CrtpPacket> mInQueue;

    private BluetoothAdapter mAdapter;
    private BluetoothSocket mSocket;
    private ReceiveThread mReceiveThread;
    private PostThread mPostThread;
    private volatile boolean mConnected;

    public BtSppLink(Context context) {
        this.mContext = context.getApplicationContext();
        this.mInQueue = new LinkedBlockingQueue<CrtpPacket>();
    }

    @Override
    public void connect() throws IOException {
        if (mConnected) {
            throw new IllegalStateException("Connection already started");
        }
        mAdapter = BluetoothAdapter.getDefaultAdapter();
        if (mAdapter == null || !mAdapter.isEnabled()) {
            notifyConnectionFailed("Bluetooth needs to be enabled");
            return;
        }
        notifyConnectionRequested();

        BluetoothDevice device = findDroneDevice();
        if (device == null) {
            notifyConnectionFailed("No paired \"" + DRONE_DEVICE_NAME + "\" found. Pair it in system settings first.");
            return;
        }
        try {
            // Cancel discovery first: it slows the RFCOMM handshake to a crawl and can
            // make the first few packets after connect time out.
            mAdapter.cancelDiscovery();
            mSocket = device.createRfcommSocketToServiceRecord(SPP_UUID);
            mSocket.connect();
            mReceiveThread = new ReceiveThread(mSocket);
            mReceiveThread.start();
            mPostThread = new PostThread(mSocket);
            mPostThread.start();
            mConnected = true;
            notifyConnected();
        } catch (IOException e) {
            closeSocket();
            notifyConnectionFailed("SPP connect failed: " + e.getMessage());
        }
    }

    /**
     * SPP has no scan, so the drone has to be paired already. Look for the ESP32 by
     * name among the bonded devices, falling back to any bonded device if the name
     * was customised in firmware — a wrong pick fails the handshake rather than
     * silently talking to the wrong thing.
     */
    private BluetoothDevice findDroneDevice() {
        Set<BluetoothDevice> bonded = mAdapter.getBondedDevices();
        if (bonded == null || bonded.isEmpty()) {
            return null;
        }
        List<BluetoothDevice> all = new ArrayList<BluetoothDevice>(bonded);
        for (BluetoothDevice d : all) {
            if (DRONE_DEVICE_NAME.equals(d.getName())) {
                return d;
            }
        }
        return all.get(0);
    }

    @Override
    public void disconnect() {
        mConnected = false;
        if (mReceiveThread != null) {
            mReceiveThread.interrupt();
            mReceiveThread = null;
        }
        if (mPostThread != null) {
            mPostThread.interrupt();
            mPostThread = null;
        }
        closeSocket();
        notifyDisconnected();
    }

    private void closeSocket() {
        if (mSocket != null) {
            try {
                mSocket.close();
            } catch (IOException e) {
                mLogger.warn("Error closing SPP socket: " + e.getMessage());
            }
            mSocket = null;
        }
    }

    @Override
    public boolean isConnected() {
        return mConnected && mSocket != null && mSocket.isConnected();
    }

    @Override
    public void sendPacket(CrtpPacket packet) {
        if (mPostThread != null) {
            mPostThread.sendPacket(packet);
        }
    }

    @Override
    public CrtpPacket receivePacket(int time) {
        try {
            return mInQueue.poll((long) time, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            return null;
        }
    }

    private void connectionLost(String reason) {
        if (mConnected) {
            mConnected = false;
            notifyConnectionLost(reason);
        }
    }

    /** Low byte of the sum of every CRTP byte including the header (PROTOCOL.md section 1). */
    static byte checksum(byte[] crtp) {
        int sum = 0;
        for (byte b : crtp) {
            sum += (b & 0xff);
        }
        return (byte) sum;
    }

    private class PostThread extends Thread {
        private final BlockingQueue<CrtpPacket> mQueue = new LinkedBlockingQueue<CrtpPacket>();
        private final BluetoothSocket mSock;

        PostThread(BluetoothSocket socket) {
            mSock = socket;
        }

        void sendPacket(CrtpPacket packet) {
            mQueue.add(packet);
        }

        @Override
        public void run() {
            try {
                OutputStream out = mSock.getOutputStream();
                while (!isInterrupted() && mSock.isConnected()) {
                    CrtpPacket packet = mQueue.take();
                    byte[] crtp = packet.toByteArray();
                    byte[] frame = new byte[crtp.length + 1];
                    System.arraycopy(crtp, 0, frame, 0, crtp.length);
                    frame[crtp.length] = checksum(crtp);
                    out.write(frame);
                    out.flush();
                }
            } catch (IOException e) {
                connectionLost("SPP write failed: " + e.getMessage());
            } catch (InterruptedException e) {
                // shutting down
            }
        }
    }

    /**
     * Pulls framed packets out of the SPP byte stream.
     *
     * <p>Because SPP coalesces and splits writes freely, the reader keeps a buffer and
     * scans for the first length in [2, MAX_FRAME] whose trailing checksum validates.
     * If nothing matches and the buffer has grown past a maximum frame, one byte is
     * dropped and the scan retries — a cheap resync that recovers from a lost byte.
     */
    private class ReceiveThread extends Thread {
        private final BluetoothSocket mSock;

        ReceiveThread(BluetoothSocket socket) {
            mSock = socket;
        }

        @Override
        public void run() {
            byte[] buf = new byte[256];
            List<Byte> acc = new ArrayList<Byte>();
            try {
                InputStream in = mSock.getInputStream();
                while (!isInterrupted() && mSock.isConnected()) {
                    int n = in.read(buf);
                    if (n < 0) {
                        connectionLost("SPP closed by the drone");
                        break;
                    }
                    for (int i = 0; i < n; i++) {
                        acc.add(buf[i]);
                    }
                    drain(acc);
                }
            } catch (IOException e) {
                connectionLost("SPP read failed: " + e.getMessage());
            }
        }

        private void drain(List<Byte> acc) {
            while (acc.size() >= 2) {
                int len = findFrame(acc);
                if (len <= 0) {
                    if (acc.size() > MAX_FRAME) {
                        acc.remove(0);
                        continue;
                    }
                    return;
                }
                byte[] frame = new byte[len];
                for (int i = 0; i < len; i++) {
                    frame[i] = acc.remove(0);
                }
                byte[] crtp = new byte[len - 1];
                System.arraycopy(frame, 0, crtp, 0, crtp.length);
                try {
                    mInQueue.put(new CrtpPacket(crtp));
                } catch (InterruptedException e) {
                    return;
                }
            }
        }

        /** Length of the first valid frame at the head of the buffer, or 0 if none yet. */
        private int findFrame(List<Byte> acc) {
            int max = Math.min(acc.size(), MAX_FRAME);
            for (int len = 2; len <= max; len++) {
                byte[] candidate = new byte[len];
                for (int i = 0; i < len; i++) {
                    candidate[i] = acc.get(i);
                }
                byte[] crtp = new byte[len - 1];
                System.arraycopy(candidate, 0, crtp, 0, crtp.length);
                if (checksum(crtp) == candidate[len - 1]) {
                    return len;
                }
            }
            return 0;
        }
    }
}
