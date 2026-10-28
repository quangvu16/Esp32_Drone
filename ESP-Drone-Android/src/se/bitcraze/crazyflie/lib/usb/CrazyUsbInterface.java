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

package se.bitcraze.crazyflie.lib.usb;

import java.io.IOException;
import java.util.List;

import android.hardware.usb.UsbDevice;

/**
 * Abstract USB interface to be independent of different implementations (eg. Java/Android)
 * Giao diện USB trừu tượng để không phụ thuộc vào các cách triển khai khác nhau (ví dụ: Java/Android)
 *
 */
public interface CrazyUsbInterface {

    /**
     * Init device
     * Khởi tạo thiết bị
     *
     * @param usbVid
     * @param usbPid
     */
    public void initDevice(int usbVid, int usbPid) throws IOException, SecurityException;

    /**
     * Release UsbInterface
     * Giải phóng UsbInterface
     *
     */
    public void releaseInterface();

    /**
     * Returns the state of the USB connection
     * Trả về trạng thái của kết nối USB
     *
     * @return true if USB device is connected, else false
     * @return true nếu thiết bị USB được kết nối, ngược lại là false
     */
    public boolean isUsbConnected();

    /**
     * Send control data
     * Gửi dữ liệu điều khiển
     *
     * @param requestType
     * @param request
     * @param value
     * @param index
     * @param data
     * @return
     */
    public int sendControlTransfer(int requestType, int request, int value, int index, byte[] data);

    /**
     * Sends bulk data
     * Gửi dữ liệu bulk
     *
     * @param data
     * @param receiveData
     * @return
     */
    public int sendBulkTransfer(byte[] data, byte[] receiveData);

    /**
     * Returns a list of CrazyRadio devices currently connected to the computer
     * Trả về danh sách các thiết bị CrazyRadio hiện đang kết nối với máy tính
     *
     * @param usbVid
     * @param usbPid
     * @return
     */
    public List<UsbDevice> findDevices(int usbVid, int usbPid);

    /**
     * Returns the firmware version of the USB device
     * Trả về phiên bản firmware của thiết bị USB
     *
     * @return firmware version
     * @return phiên bản firmware
     */
    public float getFirmwareVersion();

    /**
     * Returns the serial number of the USB device
     * Trả về số sê-ri của thiết bị USB
     *
     * @return serial number
     * @return số sê-ri
     */
    public String getSerialNumber();

    public void bulkWrite(byte[] data);

    public byte[] bulkRead();

}