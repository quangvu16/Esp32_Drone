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

package se.bitcraze.crazyfliecontrol.ble;

import java.util.Arrays;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.Handler;
import androidx.annotation.RequiresApi;
import se.bitcraze.crazyflie.lib.crtp.CommanderPacket;
import se.bitcraze.crazyflie.lib.crtp.CrtpDriver;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;

@SuppressLint("NewApi")
public class BleLink extends CrtpDriver {

    private final Logger mLogger = LoggerFactory.getLogger("BLELink");

    // Set to -40 to connect only to close-by Crazyflie
    private static final int rssiThreshold = -100;

    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothLeScanner mBluetoothLeScanner;
    private BluetoothDevice mDevice;
    private BluetoothGattCharacteristic mLedChar;
    private List<BluetoothGattCharacteristic> mLedsChars;
    private BluetoothGatt mGatt;
    private BluetoothGattCharacteristic mCrtpChar;
    private BluetoothGattCharacteristic mCrtpUpChar;
    private BluetoothGattCharacteristic mCrtpDownChar;
    private Timer mScannTimer;
    private Timer mRssiTimer;

    private static final String CF_DEVICE_NAME = "Crazyflie";
    private static final String CF_LOADER_DEVICE_NAME = "Crazyflie Loader";
    // The ESP32 firmware advertises under its own name (bleInit(WIFI_AP_SSID)), so a
    // scan filtered on the stock Crazyflie name would never see it.
    private static final String ESP_DRONE_DEVICE_NAME = "ESP-Drone";

    private static final String CLIENT_CHARACTERISTIC_CONFIG = "00002902-0000-1000-8000-00805f9b34fb";

    private static final UUID CF_SERVICE = UUID.fromString("00000201-1C7F-4F9E-947B-43B7C00A9A08");
    private static final UUID CRTP = UUID.fromString("00000202-1C7F-4F9E-947B-43B7C00A9A08");
    private static final UUID CRTPUP = UUID.fromString("00000203-1C7F-4F9E-947B-43B7C00A9A08");
    private static final UUID CRTPDOWN = UUID.fromString("00000204-1C7F-4F9E-947B-43B7C00A9A08");

    private final static int REQUEST_ENABLE_BT = 1;
    protected boolean mWritten = true;
    private Activity mContext;
    private boolean mWriteWithAnswer;
    protected boolean mConnected;

    protected enum State {IDLE, CONNECTING, CONNECTED};
    protected State state = State.IDLE;

    private ScanCallback mScanCallback21;

    private final BlockingQueue<CrtpPacket> mInQueue;

    public BleLink(Activity ctx, boolean writeWithAnswer) {
        mContext = ctx;
        mWriteWithAnswer = writeWithAnswer;
        this.mInQueue = new LinkedBlockingQueue<CrtpPacket>();
    }

    private BluetoothGattCallback mGattCallback = new BluetoothGattCallback() {

        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            super.onConnectionStateChange(gatt, status, newState);
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                mLogger.debug("onConnectionStateChange: STATE_CONNECTED");
                gatt.discoverServices();
                mGatt = gatt;
                mRssiTimer = new Timer();
                mRssiTimer.schedule(rssiTask, 1000, 1000);
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                mLogger.debug("onConnectionStateChange: STATE_DISCONNECTED");
                // This is necessary to handle a disconnect on the copter side
                stopScan();
                mConnected = false;
                state = State.IDLE;
                // it should actually be notifyConnectionLost, but there is
                // no difference between a deliberate disconnect and a lost connection
                notifyDisconnected();
            } else {
                mLogger.debug("onConnectionStateChange: else: " + newState);
                stopScan();
                mConnected = false;
                state = State.IDLE;
                notifyConnectionLost("BLE connection lost");
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            super.onServicesDiscovered(gatt, status);
            if (status != BluetoothGatt.GATT_SUCCESS) {
                gatt.disconnect();
            } else {
                BluetoothGattService cfService = gatt.getService(CF_SERVICE);
                mCrtpChar = cfService.getCharacteristic(CRTP);
                mCrtpUpChar = cfService.getCharacteristic(CRTPUP);
                mCrtpDownChar = cfService.getCharacteristic(CRTPDOWN);

                gatt.setCharacteristicNotification(mCrtpDownChar, true);

                BluetoothGattDescriptor descriptor = mCrtpDownChar.getDescriptor(UUID.fromString(CLIENT_CHARACTERISTIC_CONFIG));
                descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                gatt.writeDescriptor(descriptor);

                mLogger.debug( "Connected!");

                mConnected = true;
                mWritten = false;

                state = State.CONNECTED;
                notifyConnected();
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            super.onCharacteristicWrite(gatt, characteristic, status);
            //mLogger.debug("On write called for char: " + characteristic.getUuid().toString());
            mWritten  = true;
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            super.onDescriptorWrite(gatt, descriptor, status);
            mLogger.debug("On write called for descriptor: " + descriptor.getUuid().toString());
            mWritten = true;
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            super.onCharacteristicRead(gatt, characteristic, status);
            mLogger.debug("On read call for characteristic: " + characteristic.getUuid().toString());
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            //super.onCharacteristicChanged(gatt, characteristic);
            mLogger.debug("On changed call for characteristic: " + characteristic.getUuid().toString());
            CrtpPacket packet = unpack(characteristic.getValue());
            if (packet != null) {
                mLogger.debug("Received value for characteristic: {}, length: {}", packet.toString(), packet.toByteArray().length);
                try {
                    mInQueue.put(packet);
                } catch (InterruptedException ie) {
                    //
                }
            }
        }

        @Override
        public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status){
            if (status == BluetoothGatt.GATT_SUCCESS) {
                //mLogger.debug(String.format("BLE ReadRSSI [%d]", rssi));
                int percentage = 2*(rssi+90); // based on guesstimate
                notifyLinkQualityUpdated(limit(percentage));
            }
        }
    };

    /**
     * Limit range of int between 0 and 100
     *
     * @param value number
     * @return number between 0 and 100
     */
    private int limit(int value) {
        return Math.max(0, Math.min(value, 100));
    }

    /** The ESP32 firmware advertises as "ESP-Drone"; the stock Crazyflie as "Crazyflie". */
    private static boolean isDroneDeviceName(String name) {
        return CF_DEVICE_NAME.equals(name) || ESP_DRONE_DEVICE_NAME.equals(name);
    }

    private TimerTask rssiTask = new TimerTask() {
        @Override
        public void run() {
            if (mGatt != null) {
                mGatt.readRemoteRssi();
            }
        }
    };

    @RequiresApi(18)
    private BluetoothAdapter.LeScanCallback mScanCallback18 = new BluetoothAdapter.LeScanCallback() {

        @Override
        public void onLeScan(final BluetoothDevice device, int rssi, byte[] scanRecord) {
            if (device != null && device.getName() != null) {
                mLogger.debug("Scanned device \"" + device.getName() + "\" RSSI: " + rssi);

                if (isDroneDeviceName(device.getName()) && rssi>rssiThreshold) {
                    stopScan();
                    if (mScannTimer != null) {
                        mScannTimer.cancel();
                        mScannTimer = null;
                    }
                    state = State.CONNECTING;
                    mDevice = device;
                    mContext.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mDevice.connectGatt(mContext, false, mGattCallback);
                        }
                    });
                }
            }
        }
    };

    private void scan () {
        // Filtered scan
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            // Filters are ORed: either the stock Crazyflie or the ESP32 firmware will do.
            ScanFilter cfFilter = new ScanFilter.Builder().setDeviceName(CF_DEVICE_NAME).build();
            ScanFilter espFilter = new ScanFilter.Builder().setDeviceName(ESP_DRONE_DEVICE_NAME).build();
            mBluetoothLeScanner.startScan(Arrays.asList(cfFilter, espFilter), new ScanSettings.Builder().build(), mScanCallback21);
        } else {
            mBluetoothAdapter.startLeScan(mScanCallback18);
        }
        if (mScannTimer != null) {
            mScannTimer.cancel();
        }
        mScannTimer = new Timer();
        mScannTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                stopScan();
                state = State.IDLE;
                notifyConnectionFailed("BLE connection timeout");
            }
        }, 5000);
    }

    private void stopScan() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                mBluetoothLeScanner.stopScan(mScanCallback21);
            } else {
                mBluetoothAdapter.stopLeScan(mScanCallback18);
            }
        } catch (IllegalStateException ise) {
            mLogger.error("StopScan: IllegalStateException: " + ise.getMessage());
        }
    }

    @Override
    public void connect() {
        if (state != State.IDLE) {
            throw new IllegalArgumentException("Connection already started");
        }

        final BluetoothManager bluetoothManager = (BluetoothManager) mContext.getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();

        if (mBluetoothAdapter == null || !mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            mContext.startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
            throw new IllegalArgumentException("Bluetooth needs to be started");
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            mBluetoothLeScanner = mBluetoothAdapter.getBluetoothLeScanner();
            if (mScanCallback21 == null) {
                mScanCallback21 = new ScanCallback() {
                    @Override
                    public void onScanResult(int callbackType, ScanResult result) {
                        if (result != null) {
                            BluetoothDevice device = result.getDevice();
                            int rssi = result.getRssi();
                            if (device != null && device.getName() != null) {
                                // mLogger.debug("Scanned device \"" + device.getName() + "\" RSSI: " + rssi);

                                if (isDroneDeviceName(device.getName()) && rssi > rssiThreshold) {
                                    stopScan();
                                    if (mScannTimer != null) {
                                        mScannTimer.cancel();
                                        mScannTimer = null;
                                    }
                                    state = State.CONNECTING;
                                    mDevice = device;
                                    mContext.runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            mDevice.connectGatt(mContext, false, mGattCallback);
                                        }
                                    });
                                }
                            }
                        }
                    }
                };
            }
        }
        stopScan();
        scan();
        state = State.CONNECTING;
        notifyConnectionRequested();
    }

    @Override
    public void disconnect() {
        mContext.runOnUiThread(new Runnable() {
            public void run() {
                if(mConnected) {
                    mGatt.disconnect();
                    //delay close command to fix potential NPE
                    new Handler().postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            mGatt.close();
                            mGatt = null;
                        }
                    }, 100);
                    mConnected = false;
                    stopScan();
                    if (mScannTimer != null) {
                        mScannTimer.cancel();
                        mScannTimer = null;
                    }
                    if (mRssiTimer != null) {
                        mRssiTimer.cancel();
                        mRssiTimer = null;
                    }
                    state = State.IDLE;
                    notifyDisconnected();
                }
            }
        });
    }

    @Override
    public boolean isConnected() {
        return state == State.CONNECTED;
    }

    /**
     * The ESP32 firmware's BLE pipe is full CRTP, not just the commander subset:
     * {@code BleRawCb}/{@code BleSegCb} hand every datagram to the same
     * {@code crtpReceiveRaw} the UDP path uses, so TOC, params and logs all work.
     */
    @Override
    public boolean supportsToc() {
        return true;
    }

    int ctr = 0;
    @Override
    public void sendPacket(CrtpPacket packet) {
        // The latency workaround exists because the 50 Hz commander stream can queue
        // up on a slow link. TOC, param and log traffic must not be thinned the same
        // way — a dropped half of those stalls connection setup and the tuning screen.
        if (!mWriteWithAnswer && packet instanceof CommanderPacket
                && ((ctr++)%2 == 0)) {
            return;
        }
        // Raw CRTP goes on 0x202 only when the whole datagram fits in one 20-byte
        // write (PROTOCOL.md section 8); longer ones are segmented on 0x203.
        if (packet.toByteArray().length <= 20) {
            //send normal CRTP packet
            mContext.runOnUiThread(new SendBlePacket(packet));
        } else {
            //split and send two CRTPUP packets
            sendSplitPacket(packet);
            // TODO: test with echo packet
        }
    }

    int pid = 0;
    private void sendSplitPacket(CrtpPacket packet) {
        // Segment body is 19 CRTP bytes (20 minus the control byte), so a datagram
        // goes out as [ctrl][crtp 0..18] then [ctrl][crtp 19..]. Working in CRTP
        // byte space rather than payload space is what keeps the boundary straight:
        // CRTP byte 19 is payload[18], not payload[19].
        byte[] crtp = packet.toByteArray();
        int total = crtp.length;
        int firstLen = Math.min(19, total);

        byte[] firstPacket = new byte[firstLen + 1];
        firstPacket[0] = new ControlByte(true, pid, total).toByte();
        System.arraycopy(crtp, 0, firstPacket, 1, firstLen);
        // send first packet
        mContext.runOnUiThread(new SendBlePacket(firstPacket, mCrtpUpChar));

        // controlbyte + remaining CRTP bytes (the firmware ignores the length field
        // on a continuation, but PROTOCOL.md section 8 says it still carries the total)
        int rest = total - firstLen;
        byte[] secondPacket = new byte[rest + 1];
        secondPacket[0] = new ControlByte(false, pid, total).toByte();
        System.arraycopy(crtp, firstLen, secondPacket, 1, rest);
        // send second packet
        mContext.runOnUiThread(new SendBlePacket(secondPacket, mCrtpUpChar));
        pid = (pid+1)%4;
    }

    @Override
    public CrtpPacket receivePacket(int time) {
        try {
            return mInQueue.poll((long) time, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            mLogger.error("InterruptedException: " + e.getMessage());
            return null;
        }
    }

    private byte[] tempByteArray = new byte[32];
    private int tempPid = -1;
    private int tempFilled = -1;

    private CrtpPacket unpack (byte[] packet) {
        ControlByte header = new ControlByte(packet);
        int total = header.getTotalLength();
        int body = packet.length - 1;

        if (header.isStart()) {
            Arrays.fill(tempByteArray, (byte) 0);
            // Single-segment when the announced total fits in this segment's body.
            // Testing the length field alone misclassifies a 20-byte datagram: the
            // field stores total-1 so it reads 19, and a 20-byte write cannot carry
            // 20 CRTP bytes alongside its control byte. Mirrors the firmware's BleSegCb.
            if (total <= body) {
                System.arraycopy(packet, 1, tempByteArray, 0, body);
                return new CrtpPacket(tempByteArray);
            }
            System.arraycopy(packet, 1, tempByteArray, 0, body);
            tempPid = header.getPid();
            tempFilled = body;
        } else {
            if (header.getPid() == tempPid && tempFilled > 0) {
                int copy = Math.min(body, tempByteArray.length - tempFilled);
                System.arraycopy(packet, 1, tempByteArray, tempFilled, copy);
                tempFilled += copy;
                return new CrtpPacket(tempByteArray);
            } else {
                tempPid = -1;
                tempFilled = 0;
                mLogger.debug("Bluetooth link: Error while receiving long data: PID does not match!");
            }
        }
        return null;
    }

    private class SendBlePacket implements Runnable {
        byte[] ba;
        BluetoothGattCharacteristic characteristic;

        public SendBlePacket(byte[] ba, BluetoothGattCharacteristic characteristic) {
            this.ba = ba;
            this.characteristic = characteristic;
        }

        /**
         * Sends packet with CRTP characteristic
         *
         * @param packet
         */
        public SendBlePacket(CrtpPacket packet){
            this(packet.toByteArray(), mCrtpChar);
        }

        public void run() {
            if (characteristic == null) {
                mLogger.debug("characteristic is null!!");
                return;
            }
            if(mConnected && mWritten) {
                if (mWriteWithAnswer) {
                    characteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
                    mWritten = false;
                } else {
                    characteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
                    mWritten = true;
                }
                characteristic.setValue(ba);
                if (mGatt != null) {
                    mGatt.writeCharacteristic(characteristic);
                } else {
                    mLogger.debug("mGatt is null!!");
                }
            }
        }
    }

    private class ControlByte {

        boolean start = false;
        int pid = -1;
        int length = -1;

        /** Parses a wire control byte. The length field is total CRTP length - 1. */
        public ControlByte(byte[] array) {
            this.start = (array[0]&0x80) != 0;
            this.pid = (array[0]>>5)&0x03;
            this.length = (array[0]&0x1F) + 1;
        }

        /** Builds a control byte; {@code length} is the total CRTP length. */
        public ControlByte(boolean start, int pid, int length) {
            this.start = start;
            this.pid = pid;
            this.length = length;
        }

        public byte toByte() {
            int b = (start ? 0x80:0x00) | ((pid&0x03)<<5) | ((length-1)&0x1f);
            return (byte) b;
        }

        public boolean isStart() {
            return start;
        }

        public int getPid() {
            return pid;
        }

        /** Total CRTP length of the datagram, never the raw wire field. */
        public int getTotalLength() {
            return length;
        }

    }

}
