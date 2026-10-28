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

package se.bitcraze.crazyflie.lib.bootloader;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import se.bitcraze.crazyflie.lib.Utilities;
import se.bitcraze.crazyflie.lib.bootloader.Target.TargetTypes;
import se.bitcraze.crazyflie.lib.crazyradio.ConnectionData;
import se.bitcraze.crazyflie.lib.crazyradio.Crazyradio;
import se.bitcraze.crazyflie.lib.crazyradio.RadioDriver;
import se.bitcraze.crazyflie.lib.crtp.CrtpDriver;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket.Header;
import se.bitcraze.crazyflie.lib.crtp.CrtpPort;

/**
 * Crazyflie radio bootloader for flashing firmware.
 * Bootloader radio của Crazyflie dùng để nạp firmware.
 *
 * Bootloader utility for the Crazyflie
 * Tiện ích bootloader cho Crazyflie
 *
 */
//TODO: fix resetToBootloader methods
//TODO: sửa các phương thức resetToBootloader
//TODO: fix callbacks
//TODO: sửa các callback
//TODO: use retryCounter
//TODO: dùng retryCounter
//TODO: new Address
//TODO: địa chỉ mới
public class Cloader {

    final Logger mLogger = LoggerFactory.getLogger("Cloader");

    private CrtpDriver mDriver;
    private List<ConnectionData> mAvailableBootConnections = new ArrayList<ConnectionData>();

    private Map<Integer, Target> mTargets = new HashMap<Integer, Target>();
    private String mErrorMessage = "";
    private int mProtocolVersion = 0xFF;

    private boolean mCancelled = false;

    // Bootloader commands
    public final static int GET_INFO = 0x10;
    public final static int SET_ADDRESS = 0x11; // Only implemented on Crazyflie version 0x00
    // Chỉ được triển khai trên Crazyflie phiên bản 0x00
    public final static int GET_MAPPING = 0x12; // Only implemented in version 0x10 target 0xFF
    // Chỉ được triển khai ở phiên bản 0x10 với target 0xFF
    public final static int LOAD_BUFFER = 0x14;
    public final static int WRITE_FLASH = 0x18;
    public final static int READ_FLASH = 0x1C;


    /**
     * Init the communication class by starting to communicate with the link given.
     * clink is the link address used after resetting to the bootloader.
     * Khởi tạo tầng truyền thông bằng cách bắt đầu giao tiếp với liên kết được truyền vào.
     * clink là địa chỉ liên kết dùng sau khi reset về bootloader.
     *
     * The device is actually considered to be in firmware mode.
     * Thiết bị thực tế được xem là đang ở chế độ firmware.
     */
    // def __init__(self, link, info_cb=None, in_boot_cb=None):
    // (Python: nguyên mẫu hàm khởi tạo tương ứng)
    public Cloader(CrtpDriver driver) {
        this.mDriver = driver;

        //self._available_boot_uri = ("radio://0/110/2M", "radio://0/0/2M")
        // (Python: hai URI bootloader mặc định radio://0/110/2M và radio://0/0/2M)
        mAvailableBootConnections.add(new ConnectionData(110, Crazyradio.DR_2MPS));
        mAvailableBootConnections.add(new ConnectionData(0, Crazyradio.DR_2MPS));
    }

    /**
     * Close the link
     * Đóng liên kết
     */
    public void close() {
        if (this.mDriver != null) {
            this.mDriver.disconnect();
        }
    }

    /**
     * Scans for bootloader with the predefined channel/datarate combinations<br/>
     * Timeout is 10 seconds.
     * Quét tìm bootloader với các tổ hợp kênh/tốc độ dữ liệu định sẵn<br/>
     * Thời gian chờ là 10 giây.
     *
     * @return always the first available bootloader connection
     * @return luôn trả về kết nối bootloader đầu tiên khả dụng
     */
    public ConnectionData scanForBootloader() {
        if (!(mDriver instanceof RadioDriver)) {
            throw new IllegalArgumentException("Scanning for bootloader is only supported with a Crazyradio connection.");
        }
        long startTime = System.currentTimeMillis();
        List<ConnectionData> resultList = new ArrayList<ConnectionData>();
        while (resultList.isEmpty() && (System.currentTimeMillis() - startTime) < 10000) {
            for (ConnectionData connectionData : mAvailableBootConnections) {
                if(((RadioDriver) mDriver).scanSelected(connectionData, new byte[]{(byte) 0xFF, (byte) 0xFF, (byte) 0xFF})) {
                    resultList.add(connectionData);
                }
            }
        }
        mDriver.disconnect();

        if (!resultList.isEmpty()) {
            return resultList.get(0);
        }
        return null;
    }

    public boolean resetToBootloader(int targetId) {
        int retryCounter = 5;

        sendBootloaderPacket(new byte[]{(byte) targetId, (byte) 0xFF});

        CrtpPacket replyPk = this.mDriver.receivePacket(1);
        //while ((not pk or pk.header != 0xFF or struct.unpack("<BB", pk.data[0:2]) != (target_id, 0xFF)) and retry_counter >= 0 ):
        // (Python: điều kiện lặp gốc — chờ gói reply đúng định dạng)

        while(!isBootloaderReplyPacket(replyPk, targetId, 0xFF) /*&& retryCounter >= 0*/) {
            replyPk = this.mDriver.receivePacket(1);
            //retryCounter -= 1;
        }

        if (replyPk != null) {
            //TODO: externalise flipping of array fields
            //TODO: tách riêng bước đảo thứ tự các trường trong mảng
            // new_address = (0xb1, ) + struct.unpack("<BBBB", pk.data[2:6][::-1])
            // (Python: ghép 0xb1 với 4 byte địa chỉ đảo ngược)
            ByteBuffer bb = ByteBuffer.wrap(replyPk.getPayload()).order(ByteOrder.LITTLE_ENDIAN);
            byte[] newAddress = new byte[] {(byte) 0xb1, bb.get(5), bb.get(4), bb.get(3), bb.get(2)};

            sendBootloaderPacket(new byte[]{(byte) targetId, (byte) 0xF0, (byte) 0x00});

            //TODO: addr = int(struct.pack("B"*5, *new_address).encode('hex'), 16)
            //TODO: (Python: chuyển new_address thành địa chỉ dạng số nguyên hex)

            // Thread sleep is important here, otherwise the last packet is not sent out
            // time.sleep(0.2)
            // Việc sleep tại đây là quan trọng, nếu không gói tin cuối cùng sẽ
            // không được gửi đi (bản Python dùng time.sleep(0.2)).
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                mLogger.error("InterruptedException: " + e.getMessage());
            }
            this.mDriver.disconnect();
            //TODO: time.sleep(0.2)
            //TODO: (Python: nghỉ 0.2 giây)
            //TODO: self.link = cflib.crtp.get_link_driver("radio://0/0/2M/{}".format(addr))
            //TODO: (Python: mở lại liên kết với địa chỉ mới theo mẫu radio://0/0/2M/{addr})
            return true;
        }
        //TODO: fix dead code warning
        //TODO: sửa cảnh báo mã chết
        return false;
    }

    /**
     * Reset to the bootloader
     * Reset về bootloader
     *
     * The parameter cpuid shall correspond to the device to reset.
     * Tham số cpuid phải tương ứng với thiết bị cần reset.
     *
     * Return true if the reset has been done and the contact with the
     * bootloader is established.
     * Trả về true nếu đã reset xong và đã thiết lập được liên lạc với bootloader.
     */
    //TODO: cpu_id = target id?? NO!
    //TODO: cpu_id có bằng target id không?? KHÔNG!
    //TODO: currently not used in python cflib
    //TODO: hiện không được dùng trong python cflib
    public boolean resetToBootloader1(int cpuId) {
        /*
         * Send an echo request and wait for the answer
         * Mainly aim to bypass a bug of the crazyflie firmware that prevents
         * reset before normal CRTP communication
         * Gửi một yêu cầu echo và chờ câu trả lời.
         * Chủ yếu nhằm lách một lỗi firmware của crazyflie khiến không thể
         * reset trước khi giao tiếp CRTP bình thường.
         */
        Header header = new Header(0, CrtpPort.LINKCTRL);
        CrtpPacket pk = new CrtpPacket(header.getByte(), new byte[]{1, 2, 3, (byte) cpuId});
        this.mDriver.sendPacket(pk);

        // Wait for reply
        // Chờ phản hồi
        CrtpPacket replyPk = null;
        while(true) {
            replyPk = this.mDriver.receivePacket(2);
            if (replyPk == null) {
                return false;
            }
            if (replyPk.getHeader().getPort() == CrtpPort.LINKCTRL) {
                break;
            }
        }

        // Send the reset to bootloader request
        // Gửi yêu cầu reset về bootloader
        //pk.data = (0xFF, 0xFE) + cpu_id
        // (Python: ghép 0xFF, 0xFE với cpu_id)
        sendBootloaderPacket(new byte[]{(byte) 0xFF, (byte) 0xFE, (byte) cpuId});

        //Wait to ack the reset ...
        // Chờ ack cho lệnh reset ...
        CrtpPacket replyPk2 = null;
        while(true) {
            replyPk2 = this.mDriver.receivePacket(2);
            if (replyPk2 == null) {
                return false;
            }
            // if pk.port == 0xFF and tuple(pk.data) == (0xFF, 0xFE) + cpu_id:
            // (Python: kiểm tra gói reply có đúng định dạng)
            if(replyPk2.getHeader().getPort() == CrtpPort.ALL &&
                    replyPk2.getPayload()[0] == (byte) 0xFF &&
                    replyPk2.getPayload()[1] == (byte) 0xFE &&
                    replyPk2.getPayload()[2] == (byte) cpuId) {

                sendBootloaderPacket(new byte[]{(byte) 0xFF, (byte) 0xF0, (byte) cpuId});
                break;
            }
        }

        //time.sleep(0.1)
        // (Python: nghỉ 0.1 giây)
        this.mDriver.disconnect();
        //TODO: self.link = cflib.crtp.get_link_driver(self.clink_address)
        //TODO: (Python: mở lại liên kết với địa chỉ clink_address)
        //time.sleep(0.1)
        // (Python: nghỉ 0.1 giây)

        return updateInfo(TargetTypes.STM32); //TODO: which targetId??
        //TODO: targetId nào??
    }

    /**
     * Reset to firmware.
     * Reset về firmware.
     *
     * @param targetId
     * @return true if the reset has been done
     * @return true nếu đã reset xong
     */
    public boolean resetToFirmware(int targetId) {
        /*
         * The fake CPU ID is legacy from the Crazyflie 1.0
         * In order to reset the CPU ID had to be sent, but this
         * was removed before launching it. But the length check is
         * still in the bootloader. So to work around this bug so
         * some extra data needs to be sent.
         * fake_cpu_id = (1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12)
         * CPU ID giả là di sản từ Crazyflie 1.0: để reset thì trước đây phải
         * gửi kèm CPU ID, nhưng phần đó đã bị gỡ trước khi phát hành. Tuy
         * nhiên bước kiểm tra độ dài vẫn còn trong bootloader, nên để lách
         * lỗi này cần gửi thêm một số dữ liệu thừa.
         */

        // Send the reset to bootloader request
        // Gửi yêu cầu reset về bootloader
        //pk.data = (target_id, 0xFF) + fake_cpu_id
        // (Python: ghép target_id, 0xFF với fake_cpu_id)
        sendBootloaderPacket(new byte[]{(byte) targetId, (byte) 0xFF, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12});

        // Wait to ack the reset ...
        // Chờ ack cho lệnh reset ...
        CrtpPacket replyPk = null;
        while(true) {
            replyPk = this.mDriver.receivePacket(2);

            if (!isBootloaderReplyPacket(replyPk, targetId, 0xFF)) {
                // Difference in CF1 and CF2 (CPU ID)
                // Điểm khác nhau giữa CF1 và CF2 (CPU ID)
                byte[] data = null;
                if (targetId == TargetTypes.NRF51) { // CF2
                    data = new byte[] {(byte) targetId, (byte) 0xF0, (byte) 0x01};
                } else { // CF1
                    // pk.data = (target_id, 0xF0) + fake_cpu_id
                    // (Python: ghép target_id, 0xF0 với fake_cpu_id)
                    data = new byte[] {(byte) targetId, (byte) 0xF0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12};
                }
                sendBootloaderPacket(data);
                break;
            }
        }
        //time.sleep(0.1)
        // (Python: nghỉ 0.1 giây)
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            mLogger.error("InterruptedException: " + e.getMessage());
        }
        return true;
    }

    // FIXME: is this method really necessary?
    // FIXME: phương thức này có thực sự cần thiết không?
    public void openBootloaderConnection() throws IOException {
        if (this.mDriver.isConnected()) {
            this.mDriver.disconnect();
        }
        this.mDriver.connect();
    }

    // FIXME: is this method really necessary?
    // FIXME: phương thức này có thực sự cần thiết không?
    public void setConnectionData(ConnectionData connectionData) {
        if (mDriver instanceof RadioDriver) {
            ((RadioDriver) mDriver).setConnectionData(connectionData);
        }
    }

    /**
     * Try to get a connection with the bootloader by requesting info
     * 5 times. This let roughly 10 seconds to boot the copter ...
     * Thử thiết lập kết nối với bootloader bằng cách yêu cầu info
     * 5 lần. Việc này cho phép khoảng 10 giây để copter khởi động ...
     */
    //def check_link_and_get_info(self, target_id=0xFF):
    // (Python: nguyên mẫu hàm check_link_and_get_info)
    public boolean checkLinkAndGetInfo(int targetId) {
        for (int i = 0; i < 5; i++) {
            if (updateInfo(targetId)) {

                /*
                if self._in_boot_cb:
                    self._in_boot_cb.call(True, self.targets[target_id].protocol_version)
                if self._info_cb:
                    self._info_cb.call(self.targets[target_id])
                */
                // (Python: gọi các callback in_boot_cb và info_cb)
                if (this.mProtocolVersion != BootVersion.CF1_PROTO_VER_1) {
                    return true;
                }

                // Set radio link to a random address
                // Đặt liên kết radio vào một địa chỉ ngẫu nhiên
                /*
                addr = [0xbc] + map(lambda x: random.randint(0, 255), range(4))
                return self._set_address(addr)
                */
                // (Python: tạo địa chỉ ngẫu nhiên 5 byte bắt đầu bằng 0xbc rồi gọi _set_address)
                byte[] newAddress = new byte[5];
                newAddress[0] = (byte) 0xbc;
                for (int n = 1; n < 5; n++) {
                    newAddress[n] = (byte) ((Math.random() * 1000) % 255);  //TODO: test
                    //TODO: cần kiểm tra
                }
                //return setAddress(newAddress);
                return true;
            }

            //TODO: is this necessary?
            //TODO: điều này có cần thiết không?
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                mLogger.error("InterruptedException: " + e.getMessage());
            }
        }
        return false;
    }

    /**
     * Change copter radio address.
     * This function works only with crazyradio CRTP link.
     * Đổi địa chỉ radio của copter.
     * Hàm này chỉ hoạt động với liên kết CRTP của crazyradio.
     */
    public boolean setAddress(byte[] newAddress) {
        if (!(mDriver instanceof RadioDriver)) {
            throw new IllegalArgumentException("Setting the copter's radio address is only supported with a Crazyradio connection.");
        }
        mLogger.debug("Setting bootloader radio address to " + Utilities.getHexString(newAddress));
        return ((RadioDriver) this.mDriver).setBootloaderAddress(newAddress);
    }

    public Target requestInfoUpdate(int targetId) {
        if (!this.mTargets.containsKey(targetId)) {
            updateInfo(targetId);
        }
        /*
        if self._info_cb:
            self._info_cb.call(self.targets[target_id])
        */
        // (Python: gọi callback info_cb với target tương ứng)
        return this.mTargets.get(targetId);
    }

    /**
     * Call the command getInfo and fill up the information received in the fields of the object
     * Gọi lệnh getInfo và điền thông tin nhận được vào các thuộc tính của đối tượng
     */
    private boolean updateInfo(int targetId) {
        // Call getInfo ...
        // Gọi getInfo ...
        // pk.data = (target_id, 0x10)
        // (Python: gói tin (target_id, 0x10))
        mLogger.info("Send update info packet");
        sendBootloaderPacket(new byte[]{(byte) targetId, (byte) GET_INFO});

        // Wait for the answer
        // Chờ câu trả lời
        //TODO: retryCount?
        //TODO: có cần retryCount không?
        CrtpPacket replyPk = this.mDriver.receivePacket(2000);
        while(!isBootloaderReplyPacket(replyPk, targetId, GET_INFO)) {
            replyPk = this.mDriver.receivePacket(2000);
        }

        if(replyPk != null) {
            Target target = new Target(targetId);
            target.parseData(replyPk.getPayload());
            // set protocol version only for STM32
            // chỉ đặt phiên bản giao thức cho STM32
            if (targetId == TargetTypes.STM32) {
                this.mProtocolVersion = target.getProtocolVersion();
            }
            this.mTargets.put(targetId, target);

            // Update mapping (CF 2.0 only)
            // Cập nhật bảng ánh xạ (chỉ với CF 2.0)
            if (target.getProtocolVersion() == (byte) BootVersion.CF2_PROTO_VER && targetId == TargetTypes.STM32) {
//                updateMapping(targetId);
            }
            return true;
        } else {
            mLogger.error("Payload problem");
        }
        return false;
    }

    public Integer[] updateMapping(int targetId) {
        sendBootloaderPacket(new byte[]{(byte) targetId, (byte) GET_MAPPING});

        CrtpPacket replyPk = this.mDriver.receivePacket(2);
        while (!isBootloaderReplyPacket(replyPk, targetId, GET_MAPPING)){
            replyPk = this.mDriver.receivePacket(2);
        }

        //m = pk.datat[2:]
        // (Python: lấy phần dữ liệu từ byte thứ 2)
        byte[] mappingData = Utilities.strip(replyPk.getPayload(), 2);

        if (mappingData.length % 2 != 0){
            //raise Exception("Malformed flash mapping packet")
            // (Python: ném ngoại lệ Malformed flash mapping packet)
            mLogger.error("Malformed flash mapping packet: length is not even (%s)", mappingData.length);
            //TODO: why is the length not even?
            //TODO: tại sao độ dài lại là số lẻ?
            //return new Integer[0];
        }

        /*
        self.mapping = []
        page = 0
        for i in range(len(m)/2):
            for j in range(m[2*i]):
                self.mapping.append(page)
                page += m[(2*i)+1]
        */
        // (Python: vòng lặp dựng bảng ánh xạ trang từ dữ liệu mapping)
        List<Integer> mapping = new ArrayList<Integer>();
        int page = 0;
        for (int i = 0; i < mappingData.length/2; i++) {
            for (int j = 0; j < mappingData[2*i]; j++) {
                mapping.add(page);
                page += mappingData[(2*i)+1] & 0xFF; // "& 0xFF" deals with unsigned byte
                // "& 0xFF" giúp xử lý byte không dấu
            }
        }
        return mapping.toArray(new Integer[mapping.size()]);
    }

    /**
     * Upload data into a buffer on the Crazyflie
     * Tải dữ liệu vào một bộ đệm trên Crazyflie
     */
    public void uploadBuffer(int targetId, int page, int address, byte[] buff) {
        int count = 0;
        //pk.data = struct.pack("=BBHH", target_id, 0x14, page, address)
        // (Python: đóng gói target_id, 0x14, page, address thành gói tin)
        ByteBuffer bb = ByteBuffer.allocate(31).order(ByteOrder.LITTLE_ENDIAN);
        bb.put((byte) targetId);
        bb.put((byte) LOAD_BUFFER);
        bb.putChar((char) page);
        bb.putChar((char) address);

        for (int i = 0; i < buff.length; i++) {
            bb.put(buff[i]);

            count++;

            if (isCancelled()) {
                break;
            }

            if (count > 24) {
                sendBootloaderPacket(bb.array());
                count = 0;

                //pk.data = struct.pack("=BBHH", target_id, 0x14, page, i + address + 1)
                // (Python: đóng gói gói tin với địa chỉ i + address + 1)
                //TODO: bb.clear() did not work as intended
                //TODO: bb.clear() không hoạt động như mong đợi
                bb = ByteBuffer.allocate(31).order(ByteOrder.LITTLE_ENDIAN);
                bb.put((byte) targetId);
                bb.put((byte) LOAD_BUFFER);
                bb.putChar((char) page);
                bb.putChar((char) (i + address + 1));
            }
        }
        //mLogger.debug("Sending buffer packet: " + Utilities.getHexString(bb.array()));
        sendBootloaderPacket(bb.array());
    }

    /**
     * Read back a flash page from the Crazyflie and return it
     * Đọc lại một trang flash từ Crazyflie và trả về nó
     */
    //def read_flash(self, addr=0xFF, page=0x00):
    // (Python: nguyên mẫu hàm read_flash)
    public byte[] readFlash(int addr, int page) {
        ByteBuffer buff = null;

        Target target = this.mTargets.get(addr);
        if (target != null) {
            int pageSize = target.getPageSize();
            buff = ByteBuffer.allocate(pageSize + 1);

            for (int i = 0; i < Math.ceil(pageSize / 25.0); i++) {
                CrtpPacket replyPk = null;
                int retryCounter = 5;

                while (retryCounter >= 0) {
                    ByteBuffer bb = ByteBuffer.allocate(6).order(ByteOrder.LITTLE_ENDIAN);
                    bb.put((byte) addr);
                    bb.put((byte) READ_FLASH);
                    bb.putChar((char) page);
                    bb.putChar((char) (i*25));

                    sendBootloaderPacket(bb.array());

                    //System.out.println("ByteString send: " + getHexString(pk.getPayload()) + " " + UsbLinkJava.getByteString(pk.getPayload()));

                    //TODO: why is this different than in Python?
                    //does it have something to do with the queue size??
                    //yes, the queue is filled with empty packets
                    //how can this be avoided?
                    //TODO: tại sao lại khác với bản Python?
                    //có liên quan tới kích thước hàng đợi không??
                    //đúng vậy, hàng đợi bị lấp đầy bởi các gói tin rỗng
                    //làm sao để tránh điều này?
                    while(!isBootloaderReplyPacket(replyPk, addr, READ_FLASH)) {
                        replyPk = this.mDriver.receivePacket(10);
                    }
                    if (replyPk != null) {
                        break;
                    }
                    retryCounter--;
                }
                if (retryCounter < 0) {
                    mLogger.debug("Returning null...");
                    return new byte[0];
                } else {
                    buff.put(replyPk.getPayload(), 6, replyPk.getPayload().length - 6);
                }
            }

        }
        //return buff[0:page_size]  # For some reason we get one byte extra here...
        // (Python: trả về buff[0:page_size] — vì lý do nào đó thừa ra một byte...)
        //-> because of the ceil function?
        //-> có phải vì hàm ceil không?
        return buff.array();
    }

    /**
     * Initiate flashing of data in the buffer to flash.
     * Bắt đầu ghi dữ liệu trong bộ đệm vào flash.
     */
    public boolean writeFlash(int addr, int pageBuffer, int targetPage, int pageCount) {
        /*
        #print "Write page", flashPage
        #print "Writing page [%d] and [%d] forward" % (flashPage, nPage)
        */
        // (Python: các lệnh in debug khi ghi trang)
        CrtpPacket replyPk = null;
        int retryCounter = 5;
        //#print "Flashing to 0x{:X}".format(addr)
        // (Python: in debug địa chỉ đang ghi)

        //pk.data = struct.pack("<BBHHH", addr, 0x18, page_buffer, target_page, page_count)
        // (Python: đóng gói addr, 0x18, page_buffer, target_page, page_count)
        ByteBuffer bb = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);
        bb.put((byte) addr);
        bb.put((byte) WRITE_FLASH);
        bb.putChar((char) pageBuffer);
        bb.putChar((char) targetPage);
        bb.putChar((char) pageCount);
        sendBootloaderPacket(bb.array());

        while(!isBootloaderReplyPacket(replyPk, addr, WRITE_FLASH) && retryCounter >= 0 && !isCancelled()) {
            replyPk = this.mDriver.receivePacket(1);
            //TODO: why does it not work, when the retryCounter is activated?
            //TODO: tại sao lại không hoạt động khi bật retryCounter?
//            retryCounter--;
        }

        if (retryCounter < 0) {
            mLogger.debug("RetryCounter is < 0!");
            //self.error_code = -1
            // (Python: đặt error_code = -1)
            return false;
        }

        // handle error code
        // xử lý mã lỗi
        int errorCode = replyPk.getPayload()[3];
        switch (errorCode) {
        case 1:
            mErrorMessage = "Addresses are outside of authorized boundaries";
            break;
        case 2:
            mErrorMessage = "Flash erase failed";
            break;
        case 3:
            mErrorMessage = "Flash programming failed";
            break;
        default:
            mErrorMessage = "";
            break;
        }
        if (errorCode != 0) {
          //TODO: also call listener
          //TODO: cũng cần gọi listener
          mLogger.error("%s (error code: %s)", mErrorMessage, errorCode);
        }

        return replyPk.getPayload()[2] == 1;
    }

    //decode_cpu_id has not been implemented, because it's not used anywhere
    // decode_cpu_id chưa được triển khai vì không được dùng ở đâu cả

    private boolean isCancelled() {
        return this.mCancelled;
    }

    public void cancel() {
        this.mCancelled = true;
    }

    public String getErrorMessage() {
        return this.mErrorMessage;
    }

    public int getProtocolVersion() {
        return this.mProtocolVersion;
    }

    private void sendBootloaderPacket(byte[] data) {
        Header header = new Header((byte) 0xFF);
        CrtpPacket pk = new CrtpPacket(header.getByte(), data);
        this.mDriver.sendPacket(pk);
    }

    private boolean isBootloaderReplyPacket(CrtpPacket paket, int firstByte, int secondByte) {
        if (paket == null) {
            return false;
        }
        return paket.getHeaderByte() == (byte) 0xFF && paket.getPayload()[0] == (byte) firstByte && paket.getPayload()[1] == (byte) secondByte;
    }

    public List<Target> getTargetsAsList() {
        List<Target> targets = new ArrayList<Target>();
        for (Target t : mTargets.values()) {
            targets.add(t);
        }
        return targets;
    }

    public Map<Integer, Target> getTargets() {
        return this.mTargets;
    }

}