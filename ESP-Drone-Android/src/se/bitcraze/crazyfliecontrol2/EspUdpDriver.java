package se.bitcraze.crazyfliecontrol2;

import android.content.Context;
import android.net.wifi.WifiManager;
import android.util.Log;

import androidx.lifecycle.Observer;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import se.bitcraze.crazyflie.lib.crtp.CrtpDriver;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;

public class EspUdpDriver extends CrtpDriver {
    private static final String TAG = "EspUdpDriver";

    // SoftAP defaults (PROTOCOL.md section 8). Overridable so Station mode can reach
    // the drone at esp-drone.local and any port remap the firmware was given.
    private static final int APP_PORT_DEFAULT = 2399;
    private static final int DEVICE_PORT_DEFAULT = 2390;
    private static final String DEVICE_ADDRESS_DEFAULT = "192.168.43.42";

    private final String mDeviceAddress;
    private final int mDevicePort;
    private final int mAppPort;

    private volatile boolean mConnectMark = false;
    private volatile DatagramSocket mSocket;
    private volatile ReceiveThread mReceiveThread;
    private volatile PostThread mPostThread;

    private final EspActivity mActivity;
    private final BlockingQueue<CrtpPacket> mInQueue;
    private final WifiManager mWifiManager;

    private Observer<String> mObserver = new Observer<String>(){
        @Override
        public void onChanged(String s) {
            int networkId = mWifiManager.getConnectionInfo().getNetworkId();
            if (networkId == -1) {
                disconnect();
                notifyConnectionLost("No SoftAP connection");
            } else {
                if (mConnectMark) {
                    mConnectMark = false;
                    try {
                        InetAddress deviceAddress = InetAddress.getByName(mDeviceAddress);
                        mSocket = new DatagramSocket(null);
                        mSocket.setReuseAddress(true);
                        mSocket.bind(new InetSocketAddress(mAppPort));
                        mReceiveThread = new ReceiveThread(mSocket);
                        mReceiveThread.setPacketQueue(mInQueue);
                        mReceiveThread.start();
                        mPostThread = new PostThread(mSocket, deviceAddress, mDevicePort);
                        mPostThread.start();
                        notifyConnected();
                    } catch (IOException e) {
                        if (mSocket != null) {
                            mSocket.close();
                            mSocket = null;
                        }
                        mActivity.removeBroadcastObserver(mObserver);
                        notifyConnectionFailed("Create socket failed");
                    }
                }
            }
        }
    };

    public EspUdpDriver(EspActivity activity) {
        this(activity, DEVICE_ADDRESS_DEFAULT, DEVICE_PORT_DEFAULT, APP_PORT_DEFAULT);
    }

    /**
     * @param host drone address — 192.168.43.42 on SoftAP, esp-drone.local on Station
     * @param devicePort phone → drone (2390 by default)
     * @param appPort drone → phone, which the phone binds (2399 by default)
     */
    public EspUdpDriver(EspActivity activity, String host, int devicePort, int appPort) {
        mActivity = activity;
        mWifiManager = (WifiManager) activity.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        mInQueue = new LinkedBlockingQueue<>();
        mDeviceAddress = (host == null || host.isEmpty()) ? DEVICE_ADDRESS_DEFAULT : host;
        mDevicePort = devicePort > 0 ? devicePort : DEVICE_PORT_DEFAULT;
        mAppPort = appPort > 0 ? appPort : APP_PORT_DEFAULT;
    }

    @Override
    public void connect() throws IOException {
        Log.w(TAG, "Connect()");
        if (mSocket != null) {
            throw new IllegalStateException("Connection already started");
        }

        mConnectMark = true;
        notifyConnectionRequested();
        mActivity.observeBroadcast(mActivity, mObserver);
    }

    @Override
    public void disconnect() {
        mActivity.removeBroadcastObserver(mObserver);
        if (mSocket != null) {
            mSocket.close();
            mSocket = null;
            mReceiveThread.interrupt();
            mReceiveThread.setPacketQueue(null);
            mReceiveThread = null;
            mPostThread.interrupt();
            mPostThread = null;
            notifyDisconnected();
        }
    }

    @Override
    public boolean isConnected() {
        return mSocket != null && !mSocket.isClosed();
    }

    @Override
    public void sendPacket(CrtpPacket packet) {
        if (mSocket == null || mPostThread == null) {
            return;
        }

        mPostThread.sendPacket(packet);
    }

    @Override
    public CrtpPacket receivePacket(int wait) {
        try {
            return mInQueue.poll(wait, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            Log.w(TAG, "ReceivePacket Interrupted");
            return null;
        }
    }

    private static class PostThread extends Thread {
        private BlockingQueue<CrtpPacket> mmQueue = new LinkedBlockingQueue<>();
        private DatagramSocket mmSocket;
        private InetAddress mmDevAddress;
        private final int mmDevicePort;

        PostThread(DatagramSocket socket, InetAddress devAddress, int devicePort) {
            mmSocket = socket;
            mmDevAddress = devAddress;
            mmDevicePort = devicePort;
        }

        void sendPacket(CrtpPacket packet) {
            mmQueue.add(packet);
        }

        @Override
        public void run() {
            while (!mmSocket.isClosed() && !isInterrupted()) {
                try {
                    CrtpPacket packet = mmQueue.take();
                    byte[] data = packet.toByteArray();
                    byte[] buf = new byte[data.length + 1];
                    System.arraycopy(data, 0, buf, 0, data.length);
                    int checksum = 0;
                    for (byte b : data) {
                        checksum += (b & 0xff);
                    }
                    buf[buf.length - 1] = (byte) checksum;
                    Log.w(TAG, "run: PostData: " + Arrays.toString(buf));
                    DatagramPacket udpPacket = new DatagramPacket(buf, buf.length, mmDevAddress, mmDevicePort);
                    mmSocket.send(udpPacket);
                } catch (IOException e) {
                    Log.w(TAG, "sendPacket: IOException: " + e.getMessage());
                    mmSocket.close();
                    break;
                } catch (InterruptedException e) {
                    break;
                }
            }

            Log.d(TAG, "run: PostThread End");
        }
    }

    private static class ReceiveThread extends Thread {
        private DatagramSocket mmSocket;
        private BlockingQueue<CrtpPacket> mmQueue;

        ReceiveThread(DatagramSocket socket) {
            mmSocket = socket;
        }

        void setPacketQueue(BlockingQueue<CrtpPacket> queue) {
            mmQueue = queue;
        }

        @Override
        public void run() {
            byte[] buf = new byte[1024];
            DatagramPacket udpPacket = new DatagramPacket(buf, buf.length);
            while (!mmSocket.isClosed() && !isInterrupted()) {
                try {
                    mmSocket.receive(udpPacket);
                    Log.w(TAG, "run: ReceiveData");
                    byte[] raw = udpPacket.getData();
                    byte[] data = new byte[udpPacket.getLength() - 1];
                    System.arraycopy(udpPacket.getData(), udpPacket.getOffset(), data, 0, data.length);
                    int checksum = 0;
                    for (byte b : data) {
                        checksum += (b & 0xff);
                    }
                    if (raw[udpPacket.getLength() - 1] != (byte)checksum) {
                        Log.w(TAG, "Receive Invalid packet");
                        continue;
                    }
                    CrtpPacket packet = new CrtpPacket(data);
                    if (mmQueue != null) {
                        mmQueue.add(packet);
                    }
                } catch (IOException e) {
                    Log.w(TAG, "receivePacket: IOException: " + e.getMessage());
                    mmSocket.close();
                    break;
                }
            }

            Log.d(TAG, "run: ReceiveThread End");
        }
    }
}
