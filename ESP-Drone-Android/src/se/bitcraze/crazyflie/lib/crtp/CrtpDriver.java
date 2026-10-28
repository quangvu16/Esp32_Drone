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

package se.bitcraze.crazyflie.lib.crtp;

import java.io.IOException;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;

import se.bitcraze.crazyflie.lib.crazyflie.ConnectionListener;

/**
 * CTRP Driver main class
 * This class is inherited by all the CRTP link drivers.
 * Lớp chính của trình điều khiển CRTP.
 * Mọi trình điều khiển liên kết CRTP đều kế thừa lớp này.
 *
 */
public abstract class CrtpDriver {

    protected Set<ConnectionListener> mConnectionListeners = new CopyOnWriteArraySet<ConnectionListener>();

    /**
     * Driver constructor. Throw an exception if the driver is unable to open the URI
     * Hàm tạo của trình điều khiển. Ném ngoại lệ nếu trình điều khiển không mở được URI.
     */
    public CrtpDriver() {
    }

    /**
     * Connect the driver
     * Kết nối trình điều khiển.
     *
     * @throws IOException
     */
    public abstract void connect() throws IOException;

    /**
     * Close the link
     * Đóng liên kết.
     */
    public abstract void disconnect();

    /**
     * Check whether the link is connected.
     * Kiểm tra xem liên kết đã được kết nối chưa.
     *
     * @return <code>true</code> if the link is connected.
     * @return <code>true</code> nếu liên kết đã kết nối.
     */
    public abstract boolean isConnected();

    /**
     * Send a CRTP packet
     * Gửi một gói tin CRTP.
     *
     * @param packet
     */
    public abstract void sendPacket(CrtpPacket packet);

    /**
     * Receive a CRTP packet.
     * Nhận một gói tin CRTP.
     *
     * @param wait The time to wait for a packet in milliseconds. -1 means forever
     * @param wait Thời gian chờ gói tin theo mili giây. -1 nghĩa là chờ mãi mãi
     * @return One CRTP packet or None if no packet has been received.
     * @return Một gói tin CRTP hoặc None nếu không nhận được gói tin nào.
     */
    public abstract CrtpPacket receivePacket(int wait);

    /**
     * Whether this link carries the full CRTP protocol (TOC, parameters and
     * logging) or only the flight-control subset. Connection setup downloads
     * the TOCs only for links that can answer TOC queries.
     * Liên kết này mang toàn bộ giao thức CRTP (TOC, tham số và logging)
     * hay chỉ phần điều khiển bay. Quá trình thiết lập kết nối chỉ tải TOC
     * cho những liên kết có thể trả lời truy vấn TOC.
     *
     * @return <code>true</code> if TOC-based setup should run over this link
     * @return <code>true</code> nếu phần thiết lập dựa trên TOC nên chạy qua liên kết này
     */
    public boolean supportsToc() {
        return true;
    }

    /* CONNECTION LISTENER */

    public void addConnectionListener(ConnectionListener listener) {
        this.mConnectionListeners.add(listener);
    }

    public void removeConnectionListener(ConnectionListener listener) {
        this.mConnectionListeners.remove(listener);
    }

    /**
     * Notify all registered listeners about a requested connection
     * Thông báo cho tất cả listener đã đăng ký về một yêu cầu kết nối
     */
    protected void notifyConnectionRequested(){
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.connectionRequested();
        }
    }

    /**
     * Notify all registered listeners about a connect.
     * Thông báo cho tất cả listener đã đăng ký về một kết nối.
     */
    public void notifyConnected(){
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.connected();
        }
    }
    /**
     * Notify all registered listeners about a finished setup.
     * Thông báo cho tất cả listener đã đăng ký về một lần thiết lập hoàn tất.
     */
    public void notifySetupFinished(){
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.setupFinished();
        }
    }

    /**
     * Notify all registered listeners about a failed connection attempt.
     * Thông báo cho tất cả listener đã đăng ký về một lần thử kết nối thất bại.
     *
     * @param msg
     */
    protected void notifyConnectionFailed(String msg) {
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.connectionFailed(msg);
        }
    }

    /**
     * Notify all registered listeners about a lost connection.
     * Thông báo cho tất cả listener đã đăng ký về một kết nối bị mất.
     *
     * @param msg
     */
    protected void notifyConnectionLost(String msg) {
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.connectionLost(msg);
        }
    }

    /**
     * Notify all registered listeners about a disconnect.
     * Thông báo cho tất cả listener đã đăng ký về một lần ngắt kết nối.
     */
    protected void notifyDisconnected() {
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.disconnected();
        }
    }

    /**
     * Notify all registered listeners about a link quality update.
     * Thông báo cho tất cả listener đã đăng ký về một bản cập nhật chất lượng liên kết.
     *
     * @param percent quality of the link (0 = connection lost, 100 = good)
     * @param percent chất lượng của liên kết (0 = mất kết nối, 100 = tốt)
     */
    protected void notifyLinkQualityUpdated(int percent) {
        for (ConnectionListener cl : this.mConnectionListeners) {
            cl.linkQualityUpdated(percent);
        }
    }

}
