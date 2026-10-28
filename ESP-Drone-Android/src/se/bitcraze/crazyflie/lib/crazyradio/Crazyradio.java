/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (C) 2013 Bitcraze AB
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

package se.bitcraze.crazyflie.lib.crazyradio;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;
import se.bitcraze.crazyflie.lib.usb.CrazyUsbInterface;

/**
 * Used for communication with the Crazyradio USB dongle
 * Dùng để giao tiếp với dongle USB Crazyradio
 *
 */
public class Crazyradio {

    final Logger mLogger = LoggerFactory.getLogger(this.getClass().getSimpleName());

    // USB parameters
    public final static int CRADIO_VID = 0x1915; //Vendor ID
    // ID nhà sản xuất
    public final static int CRADIO_PID = 0x7777; //Product ID
    // ID sản phẩm

    // Dongle configuration requests
    // See http://wiki.bitcraze.se/projects:crazyradio:protocol for documentation
    // Xem http://wiki.bitcraze.se/projects:crazyradio:protocol để biết tài liệu
    private final static int SET_RADIO_CHANNEL = 0x01;
    private final static int SET_RADIO_ADDRESS = 0x02;
    private final static int SET_DATA_RATE = 0x03;
    private final static int SET_RADIO_POWER = 0x04;
    private final static int SET_RADIO_ARD = 0x05;
    private final static int SET_RADIO_ARC = 0x06;
    private final static int ACK_ENABLE = 0x10;
    private final static int SET_CONT_CARRIER = 0x20;
    private final static int SCAN_CHANNELS = 0x21;
    private final static int LAUNCH_BOOTLOADER = 0xFF;

    // configuration constants
    public final static int DR_250KPS = 0;
    public final static int DR_1MPS = 1;
    public final static int DR_2MPS = 2;

    public final static int P_M18DBM = 0;
    public final static int P_M12DBM = 1;
    public final static int P_M6DBM = 2;
    public final static int P_0DBM = 3;

    private CrazyUsbInterface mUsbInterface;
    private int mArc;
    private float mVersion; // Crazyradio firmware version
    // phiên bản firmware của Crazyradio
    private String mSerialNumber; // Crazyradio serial number
    // số serial của Crazyradio

    protected final static byte[] NULL_PACKET = new byte[] { (byte) 0xff };

    /**
     * Create object and scan for USB dongle if no device is supplied
     * Tạo đối tượng và quét tìm USB dongle nếu không được truyền thiết bị
     *
     * @param usbInterface
     */
    public Crazyradio(CrazyUsbInterface usbInterface) {
        this.mUsbInterface = usbInterface;
        try {
            this.mUsbInterface.initDevice(CRADIO_VID, CRADIO_PID);
        } catch (SecurityException | IOException e) {
            mLogger.error(e.getMessage());
            return;
        }

        /*
        def __init__(self, device=None, devid=0):
            if device is None:
                try:
                    device = _find_devices()[devid]
                except Exception:
                    raise Exception("Cannot find a Crazyradio Dongle")

            self.dev = device
        */
        // (Python: bản gốc của hàm khởi tạo — tự tìm dongle Crazyradio nếu
        // không chỉ định thiết bị, báo lỗi nếu không tìm thấy)

        this.mVersion = mUsbInterface.getFirmwareVersion();

        if (this.mVersion < 0.4) {
            this.mLogger.warn("You should update to Crazyradio firmware V0.4+");
        }

        this.mSerialNumber = mUsbInterface.getSerialNumber();

        // Reset the dongle to power up settings
        // Đưa dongle về thiết lập mặc định lúc cấp nguồn
        this.mLogger.debug("Resetting dongle to power up settings...");
        setDatarate(DR_2MPS);
        setChannel(2);
        this.mArc = -1;
        if (mVersion >= 0.4) {
            setContinuousCarrier(false);
            // self.set_address((0xE7,) * 5)
            // (Python: gọi set_address với 5 byte 0xE7)
            setAddress(new byte[] {(byte) 0xE7, (byte) 0xE7, (byte) 0xE7, (byte) 0xE7, (byte) 0xE7});
            setPower(P_0DBM);
            setArc(3);
            setArdBytes(32);
        }

    }

    public void disconnect() {
        mLogger.debug("disconnect()");
        if(mUsbInterface != null) {
            mUsbInterface.releaseInterface();
        }
    }

    /* ### Dongle configuration ### */

    /**
     * Set the radio channel to be used.
     * Đặt kênh radio sẽ dùng.
     *
     * @param channel new channel. Must be in range 0-125.
     * @param channel kênh mới. Phải nằm trong khoảng 0-125.
     */
    public void setChannel(int channel) {
        if (channel < 0 || channel > 125) {
            throw new IllegalArgumentException("Channel must be an integer value between 0 and 125");
        }
        sendVendorSetup(SET_RADIO_CHANNEL, channel, 0, null);
    }

    /**
     * Set the radio address to be used.
     * Đặt địa chỉ radio sẽ dùng.
     * The same address must be configured in the receiver for the communication to work.
     * Địa chỉ giống nhau phải được cấu hình ở phía bộ thu thì giao tiếp mới hoạt động.
     *
     * @param address new address with a length of 5 byte.
     * @param address địa chỉ mới gồm 5 byte.
     * @throws IllegalArgumentException if the length of the address doesn't equal 5 bytes
     * @throws IllegalArgumentException nếu độ dài địa chỉ khác 5 byte
     */
    public void setAddress(byte[] address) {
        if (address.length != 5) {
            throw new IllegalArgumentException("Radio address must be 5 bytes long");
        }
        sendVendorSetup(SET_RADIO_ADDRESS, 0, 0, address);
    }

    /**
     * Set the radio data rate to be used.
     * Đặt tốc độ dữ liệu radio sẽ dùng.
     *
     * @param datarate new data rate. Possible values are in range 0-2.
     * @param datarate tốc độ dữ liệu mới. Giá trị có thể nằm trong khoảng 0-2.
     */
    public void setDatarate(int datarate) {
        if (datarate < 0 || datarate > 2) {
            throw new IllegalArgumentException("Data rate must be an int value between 0 and 2");
        }
        sendVendorSetup(SET_DATA_RATE, datarate, 0, null);
    }

    /**
     * Set the radio power to be used
     * Đặt công suất radio sẽ dùng
     *
     * @param power new radio power
     * @param power công suất radio mới
     */
    public void setPower(int power) {
        //TODO: add argument check
        //TODO: cần kiểm tra đối số
        sendVendorSetup(SET_RADIO_POWER, power, 0, null);
    }

    /**
     * Set the ACK retry count for radio communication
     * Đặt số lần gửi lại cho truyền thông radio
     *
     * Set how often the radio will retry a transfer if the ACK has not been received.
     * Đặt số lần radio sẽ gửi lại một lần truyền nếu chưa nhận được ACK.
     *
     * @param arc number of retries.
     * @param arc số lần gửi lại.
     * @throws IllegalArgumentException if the number of retries is not in range 0-15.
     * @throws IllegalArgumentException nếu số lần gửi lại không nằm trong khoảng 0-15.
     */
    public void setArc(int arc) {
        if (arc < 0 || arc > 15) {
            throw new IllegalArgumentException("Count must be in range 0-15");
        }
        sendVendorSetup(SET_RADIO_ARC, arc, 0, null);
        this.mArc = arc;
    }

    /**
     * Set the ACK retry delay for radio communication
     * Đặt độ trễ gửi lại cho truyền thông radio
     *
     * Configure the time the radio waits for the acknowledge.
     * Cấu hình thời gian radio chờ nhận ack.
     *
     * @param us microseconds to wait. Will be rounded to the closest possible value supported by the radio.
     * @param us số micro giây cần chờ. Sẽ được làm tròn tới giá trị gần nhất mà radio hỗ trợ.
     */
    public void setArdTime(int us) {
        /*
        # Auto Retransmit Delay:
        # 0000 - Wait 250uS
        # 0001 - Wait 500uS
        # 0010 - Wait 750uS
        # ........
        # 1111 - Wait 4000uS
        # Bảng độ trễ gửi lại tự động: 0000 ứng với mức chờ 250uS, mỗi bước
        # tăng thêm 250uS, tới 1111 ứng với mức chờ 4000uS.
        */
        // Round down, to value representing a multiple of 250uS
        // Làm tròn xuống tới giá trị đại diện cho một bội số của 250uS
        int t = (int) Math.round(us / 250.0) - 1;
        if (t < 0) {
            t = 0;
        } else if (t > 0xF) {
            t = 0xF;
        }
        sendVendorSetup(SET_RADIO_ARD, t, 0, null);
    }

    /**
     * Set the length of the ACK payload.
     * Đặt độ dài của phần dữ liệu trong ACK.
     *
     * @param nbytes number of bytes in the payload.
     * @param nbytes số byte trong phần dữ liệu.
     * @throws IllegalArgumentException if the payload length is not in range 0-32.
     * @throws IllegalArgumentException nếu độ dài phần dữ liệu không nằm trong khoảng 0-32.
     */
    public void setArdBytes(int nbytes) {
        if (nbytes < 0 || nbytes > 32) {
            throw new IllegalArgumentException("Payload length must be in range 0-32");
        }
        sendVendorSetup(SET_RADIO_ARD, 0x80 | nbytes, 0, null);
    }

    /**
     * Set the continuous carrier mode. When enabled the radio chip provides a
     * test mode in which a continuous non-modulated sine wave is emitted. When
     * this mode is activated the radio dongle does not transmit any packets.
     * Đặt chế độ sóng mang liên tục. Khi bật, chip radio cung cấp chế độ thử
     * nghiệm phát sóng sine liên tục không điều biến. Khi chế độ này được kích
     * hoạt, dongle radio không truyền gói tin nào.
     *
     * @param active <code>true</code> to enable the continuous carrier mode
     * @param active <code>true</code> để bật chế độ sóng mang liên tục
     */
    public void setContinuousCarrier(boolean active) {
        sendVendorSetup(SET_CONT_CARRIER, (active ? 1 : 0), 0, null);
    }

    private boolean hasFwScan() {
        /*
          #return self.version >= 0.5
          return mUsbInterface.getFirmwareVersion() > 0.5;
          # FIXME: Mitigation for Crazyradio firmware bug #9
          # FIXME: biện pháp khắc phục lỗi firmware của Crazyradio #9
        */
        return false;
    }

    /**
     * Scan all channels between 0 and 125
     * Quét toàn bộ các kênh trong khoảng từ 0 đến 125
     *
     * @return list of channels
     * @return danh sách các kênh
     */
    public List<Integer> scanChannels1() {
        return scanChannels(0, 125);
    }

    public List<Integer> scanChannels(int start, int stop) {
        List<Integer> result = new ArrayList<Integer>();

        if (mUsbInterface != null && mUsbInterface.isUsbConnected()) {
            if (hasFwScan()) {
                result.addAll(firmwareScan(start, stop));
            } else {
                // Slow PC-driven scan
                // Quét chậm do PC điều khiển
                mLogger.debug("Slow scan...");
                // for i in range(start, stop + 1):
                // (Python: vòng lặp quét từng kênh trong khoảng)
                for (int channel = start; channel <= stop; channel++) {
                    if(scanSelected(channel, NULL_PACKET)) {
                        mLogger.debug("Found channel: " + channel);
                        result.add(channel);
                    }
                    try {
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        mLogger.error("InterruptedException: " + e.getMessage());
                    }
                }
            }
        } else {
            mLogger.warn("Crazyradio not attached.");
        }
        return result;
    }

    /**
     * Scan for available channels.
     * Quét các kênh khả dụng.
     *
     * @return array containing the found channels and data rates.
     * @return mảng chứa các kênh tìm được và tốc độ dữ liệu.
     * @throws IOException if the Crazyradio is not attached (the connection is <code>null</code>).
     * @throws IOException nếu Crazyradio chưa được gắn vào (kết nối là <code>null</code>).
     */

    public ConnectionData[] scanChannels() throws IOException {
        return scanChannels(true);
    }

    /**
     * Scan for available channels.
     * Quét các kênh khả dụng.
     *
     * @param useSlowScan if true, then the slow scan is used
     * @param useSlowScan nếu true thì dùng phép quét chậm
     * @return array containing the found channels and datarates.
     * @return mảng chứa các kênh tìm được và tốc độ dữ liệu.
     * @throws IOException if the Crazyradio is not attached (the connection is <code>null</code>).
     * @throws IOException nếu Crazyradio chưa được gắn vào (kết nối là <code>null</code>).
     */
    public ConnectionData[] scanChannels(boolean useSlowScan) throws IOException {
        List<ConnectionData> result = new ArrayList<ConnectionData>();
        if (mUsbInterface.isUsbConnected()) {
            // null packet
            // gói tin null
            final byte[] packet = CrtpPacket.NULL_PACKET.toByteArray();
            final byte[] rdata = new byte[64];

            mLogger.debug("Scanning...");
            // scan for all 3 data rates
            // quét với cả 3 tốc độ dữ liệu
            for (int datarate = 0; datarate < 3; datarate++) {
                // set data rate
                // đặt tốc độ dữ liệu
                mUsbInterface.sendControlTransfer(0x40, SET_DATA_RATE, datarate, 0, null);
                if (useSlowScan) {
                    result.addAll(scanChannelsSlow(datarate));
                } else {
                    mLogger.debug("Fast firmware scan...");
                    //long transfer timeout (1000) is important!
                    // thời gian chờ cho lần truyền dài (1000) là rất quan trọng!
                    mUsbInterface.sendControlTransfer(0x40, SCAN_CHANNELS, 0, 125, packet);
                    final int nfound = mUsbInterface.sendControlTransfer(0xc0, SCAN_CHANNELS, 0, 0, rdata);
                    if (nfound != 64) {
                        for (int i = 0; i < nfound; i++) {
                            result.add(new ConnectionData(rdata[i], datarate));
                            mLogger.debug("Found channel: " + rdata[i] + " Data rate: " + datarate);
                        }
                    }
                }
            }
        } else {
            mLogger.debug("connection is null");
            throw new IllegalStateException("Crazyradio not attached");
        }
        return result.toArray(new ConnectionData[result.size()]);
    }

    /**
     * Slow manual scan
     * Quét thủ công chậm
     *
     * @param datarate the data rate that should be used during the scan
     * @param datarate tốc độ dữ liệu cần dùng trong khi quét
     * @throws IOException if the Crazyradio is not attached (the connection is <code>null</code>).
     * @throws IOException nếu Crazyradio chưa được gắn vào (kết nối là <code>null</code>).
     */
    private List<ConnectionData> scanChannelsSlow(int datarate) throws IOException {
        mLogger.debug("Slow manual scan...");
        List<ConnectionData> result = new ArrayList<ConnectionData>();

        for (int channel = 0; channel < 126; channel++) {
            // set channel
            // đặt kênh
            mUsbInterface.sendControlTransfer(0x40, SET_RADIO_CHANNEL, channel, 0, null);

            byte[] receiveData = new byte[33];
            final byte[] sendData = CrtpPacket.NULL_PACKET.toByteArray();

            mUsbInterface.sendBulkTransfer(sendData, receiveData);
            if ((receiveData[0] & 1) != 0) { // check if ack received
            // kiểm tra xem đã nhận được ack chưa
                result.add(new ConnectionData(channel, datarate));
                mLogger.debug("Channel found: " + channel + " Data rate: " + datarate);
            }
            try {
                Thread.sleep(20, 0);
            } catch (InterruptedException e) {
                mLogger.error("scanChannelsSlow InterruptedException");
            }
        }
        return result;
    }

    public boolean scanSelected(int channel, int datarate, byte[] packet) {
        setDatarate(datarate);
        return scanSelected(channel, packet);
    }

    private boolean scanSelected(int channel, byte[] packet) {
        setChannel(channel);
        RadioAck status = sendPacket(packet);
        return (status != null && status.isAck());
    }


    /* ### Data transfers ### */

    private List<Integer> firmwareScan(int start, int stop) {
        mLogger.debug("Fast scan...");
        List<Integer> result = new ArrayList<Integer>();
        final byte[] rdata = new byte[64];
        mUsbInterface.sendControlTransfer(0x40, SCAN_CHANNELS, start, stop, NULL_PACKET);
        final int nfound = mUsbInterface.sendControlTransfer(0xc0, SCAN_CHANNELS, 0, 0, rdata);
        if (nfound != 64) {
            for (int i = 0; i < nfound; i++) {
                result.add((int) rdata[i]);
                mLogger.debug("Found channel: " + rdata[i]);
            }
        }
        return result;
    }

    /**
     * Send a packet and receive the ack from the radio dongle.
     * The ack contains information about the packet transmission
     * and a data payload if the ack packet contained any
     * Gửi một gói tin và nhận ack từ dongle radio. Ack chứa thông tin về
     * quá trình truyền gói tin, kèm phần dữ liệu nếu gói tin ack có chứa.
     *
     * @param dataOut bytes to send
     * @param dataOut các byte cần gửi
     */
    public RadioAck sendPacket(byte[] dataOut) {
        RadioAck ackIn = null;
        byte[] data = new byte[33]; // 33?

        if (mUsbInterface == null || !mUsbInterface.isUsbConnected()) {
            return null;
        }
        mUsbInterface.sendBulkTransfer(dataOut, data);

        // if data is not None:
        // (Python: chỉ xử lý khi data khác None)
        ackIn = new RadioAck();
        if (data[0] != 0) {
            ackIn.setAck((data[0] & 0x01) != 0);
            ackIn.setPowerDet((data[0] & 0x02) != 0);
            ackIn.setRetry(data[0] >> 4);
            ackIn.setData(Arrays.copyOfRange(data, 1, data.length));
        } else {
            ackIn.setRetry(mArc);
        }
        return ackIn;
    }

    private void sendVendorSetup(int request, int value, int index, byte[] data) {
        // usb.TYPE_VENDOR = 64 <=> 0x40
        // usb.TYPE_VENDOR = 64 tương ứng 0x40
        int usbTypeVendor = 0x40;
        mUsbInterface.sendControlTransfer(usbTypeVendor, request, value, index, data);
    }

    public float getVersion() {
        return this.mVersion;
    }

    public String getSerialNumber() {
        return this.mSerialNumber;
    }

}
