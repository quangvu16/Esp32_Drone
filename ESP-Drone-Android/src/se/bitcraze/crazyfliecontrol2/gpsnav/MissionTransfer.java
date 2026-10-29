package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;
import se.bitcraze.crazyflie.lib.crazyflie.DataListener;
import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;
import se.bitcraze.crazyflie.lib.crtp.CrtpPort;

/**
 * The mission-transfer side of PROTOCOL.md §12: runs the MISSION_* handshake
 * over CRTP port 7 channel 2 and re-sends its last message every 500 ms until
 * the expected reply arrives. The drone's handlers are idempotent, so those
 * repeats are how a lossy UDP hop stays correct.
 * <p>
 * Phía truyền nhiệm vụ của PROTOCOL.md mục 12: chạy bắt tay MISSION_* trên CRTP
 * port 7 kênh 2 và gửi lại thông điệp cuối mỗi 500 ms cho tới khi nhận được câu
 * trả lời mong đợi. Xử lý phía drone lặp được, nên các lần gửi lại đó là cách
 * một đường truyền UDP hay mất gói vẫn đúng.
 *
 * <p>Upload:  COUNT n → REQUEST 0 → ITEM 0 → REQUEST 1 → … → ITEM n-1 → ACK 0.
 * Download:  REQUEST_LIST → COUNT n → REQUEST 0 → ITEM 0 → … → ITEM n-1.
 * <p>All state lives on the main thread; the UDP receive thread only posts in.
 * Mọi trạng thái nằm ở luồng chính; luồng nhận UDP chỉ chuyển vào.
 */
public class MissionTransfer {

    private static final String TAG = "MissionTransfer";

    /** PROTOCOL.md §12: the app repeats its last mission message at this rate. */
    /** PROTOCOL.md mục 12: app lặp thông điệp nhiệm vụ cuối với nhịp này. */
    private static final long RESEND_MS = 500;

    /** Give up after this many unanswered repeats (5 s of silence). */
    /** Bỏ cuộc sau số lần lặp không được trả lời này (5 giây im lặng). */
    private static final int MAX_TRIES = 10;

    public interface Listener {
        void onTransferStatus(String message);

        void onUploadFinished(int result);

        void onDownloadFinished(PlannerMission mission, int result);

        /** cmd is one of CMD_SET_CURRENT / CMD_START; ok is the status byte. */
        /** cmd là một trong CMD_SET_CURRENT / CMD_START; ok là byte status. */
        void onCommandFinished(int cmd, boolean ok);

        /** Async MISSION_CURRENT telemetry from the drone. */
        /** Telemetry MISSION_CURRENT bất đồng bộ từ drone. */
        void onCurrentItem(int seq);
    }

    private static final int MODE_IDLE = 0;
    private static final int MODE_UPLOAD = 1;
    private static final int MODE_DOWNLOAD = 2;
    private static final int MODE_SIMPLE = 3;

    private final Handler mHandler = new Handler(Looper.getMainLooper());
    private final Listener mListener;

    private Crazyflie mLink;
    private DataListener mDataListener;

    private int mMode = MODE_IDLE;
    private MissionPacket mLastSent;
    private int mTries;

    private List<PlannerItem> mUpItems;
    private int mUpTotal;

    private PlannerMission mDownMission;
    private int mDownTotal;

    private int mSimpleCmd;

    public MissionTransfer(Listener listener) {
        this.mListener = listener;
    }

    /** Start listening on the link; safe to call again with the same link. */
    /** Bắt đầu nghe trên link; gọi lại với cùng link vẫn an toàn. */
    public void attach(Crazyflie link) {
        if (link == mLink && mDataListener != null) {
            return;
        }
        detach();
        mLink = link;
        if (mLink == null) {
            return;
        }
        mDataListener = new DataListener(CrtpPort.COMMANDER_GENERIC) {
            @Override
            public void dataReceived(final CrtpPacket packet) {
                // The listener is per port; navigation lives on channel 1 of
                // the same port. Only channel 2 is the mission channel.
                // Trình nghe theo cổng; định hướng nằm ở kênh 1 của cùng cổng.
                // Chỉ kênh 2 là kênh nhiệm vụ.
                if (packet.getHeader().getChannel() != MissionPacket.CHANNEL) {
                    return;
                }
                final byte[] payload = packet.getPayload();
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        onReply(payload);
                    }
                });
            }
        };
        mLink.addDataListener(mDataListener);
    }

    /** Stop listening and abandon any transfer in flight. */
    /** Nghe ngừng và hủy mọi phiên truyền đang chạy. */
    public void detach() {
        mHandler.removeCallbacks(mResend);
        if (mLink != null && mDataListener != null) {
            mLink.removeDataListener(mDataListener);
        }
        mDataListener = null;
        mLink = null;
        mMode = MODE_IDLE;
        mLastSent = null;
    }

    public boolean isBusy() {
        return mMode != MODE_IDLE;
    }

    /** Upload the items to the drone's store (an empty list clears it). */
    /** Tải các mục lên kho của drone (danh sách rỗng = xóa). */
    public void upload(List<PlannerItem> items) {
        if (isBusy()) {
            fail("Another transfer is still running");
            return;
        }
        if (mLink == null) {
            fail("No drone connected");
            return;
        }
        mUpItems = new ArrayList<>(items);
        mUpTotal = mUpItems.size();
        mMode = MODE_UPLOAD;
        mListener.onTransferStatus(mUpTotal == 0
                ? "Clearing stored mission"
                : "Uploading 0/" + mUpTotal);
        send(MissionPacket.count(mUpTotal));
    }

    /** Download whatever the drone holds into a fresh mission. */
    /** Tải về nội dung drone đang giữ thành một nhiệm vụ mới. */
    public void download() {
        if (isBusy()) {
            fail("Another transfer is still running");
            return;
        }
        if (mLink == null) {
            fail("No drone connected");
            return;
        }
        mMode = MODE_DOWNLOAD;
        mDownMission = new PlannerMission();
        mListener.onTransferStatus("Requesting mission list");
        send(MissionPacket.requestList());
    }

    /** Jump to an item; the drone answers {@code CMD_SET_CURRENT} + status. */
    /** Nhảy tới một mục; drone trả {@code CMD_SET_CURRENT} + status. */
    public void setCurrent(int seq) {
        startSimple(MissionPacket.setCurrent(seq), "Setting current item");
    }

    /** Fly the stored mission; the drone answers {@code CMD_START} + status. */
    /** Bay nhiệm vụ đã lưu; drone trả {@code CMD_START} + status. */
    public void startMission() {
        startSimple(MissionPacket.start(), "Starting mission");
    }

    private void startSimple(MissionPacket packet, String status) {
        if (isBusy()) {
            fail("Another transfer is still running");
            return;
        }
        if (mLink == null) {
            fail("No drone connected");
            return;
        }
        mMode = MODE_SIMPLE;
        mSimpleCmd = packet.getCmd();
        mListener.onTransferStatus(status);
        send(packet);
    }

    // ---- state machine, main thread only ---------------------------------

    private void send(MissionPacket packet) {
        mLastSent = packet;
        mTries = 0;
        mLink.sendPacket(packet);
        mHandler.removeCallbacks(mResend);
        mHandler.postDelayed(mResend, RESEND_MS);
    }

    private final Runnable mResend = new Runnable() {
        @Override
        public void run() {
            if (mLastSent == null || mLink == null) {
                return;
            }
            if (mTries >= MAX_TRIES) {
                // Stop before the repeat becomes its own hazard: the drone's
                // handlers are idempotent, but a dead link will never answer.
                // Dừng trước khi việc lặp tự gây hại: xử lý phía drone lặp được,
                // nhưng link chết sẽ không bao giờ trả lời.
                int mode = mMode;
                mLastSent = null;
                mMode = MODE_IDLE;
                if (mode == MODE_UPLOAD) {
                    mListener.onUploadFinished(MissionPacket.MISSION_ERROR);
                } else if (mode == MODE_DOWNLOAD) {
                    mListener.onDownloadFinished(null, MissionPacket.MISSION_ERROR);
                } else if (mode == MODE_SIMPLE) {
                    mListener.onCommandFinished(mSimpleCmd, false);
                }
                mListener.onTransferStatus("No reply from the drone");
                return;
            }
            mTries++;
            Log.d(TAG, "resend try " + mTries + ": " + mLastSent);
            mLink.sendPacket(mLastSent);
            mHandler.postDelayed(this, RESEND_MS);
        }
    };

    private void onReply(byte[] payload) {
        if (payload.length < 1) {
            return;
        }
        int cmd = payload[0] & 0xFF;

        // MISSION_CURRENT telemetry is welcome in any state.
        // Telemetry MISSION_CURRENT được chào đón ở mọi trạng thái.
        if (cmd == MissionPacket.CMD_CURRENT) {
            int seq = MissionPacket.parseU16(payload, 1);
            if (seq >= 0) {
                mListener.onCurrentItem(seq);
            }
            return;
        }

        if (mMode == MODE_UPLOAD && mLastSent != null
                && mLastSent.getCmd() == MissionPacket.CMD_COUNT) {
            if (cmd == MissionPacket.CMD_REQUEST) {
                int seq = MissionPacket.parseU16(payload, 1);
                if (seq < 0 || seq >= mUpTotal) {
                    finishUpload(MissionPacket.MISSION_INVALID);
                } else {
                    send(MissionPacket.item(mUpItems.get(seq), seq));
                }
            } else if (cmd == MissionPacket.CMD_ACK) {
                // COUNT with 0 items clears the store and ACKs immediately.
                // COUNT với 0 mục xóa kho và ACK ngay.
                finishUpload(MissionPacket.parseU8(payload, 1));
            }
            return;
        }

        if (mMode == MODE_UPLOAD && mLastSent != null
                && mLastSent.getCmd() == MissionPacket.CMD_ITEM) {
            if (cmd == MissionPacket.CMD_REQUEST) {
                int seq = MissionPacket.parseU16(payload, 1);
                if (seq < 0 || seq >= mUpTotal) {
                    finishUpload(MissionPacket.MISSION_INVALID);
                } else {
                    mListener.onTransferStatus("Uploading " + seq + "/" + mUpTotal);
                    send(MissionPacket.item(mUpItems.get(seq), seq));
                }
            } else if (cmd == MissionPacket.CMD_ACK) {
                int type = MissionPacket.parseU8(payload, 1);
                if (type == MissionPacket.MISSION_ACCEPTED) {
                    mListener.onTransferStatus("Mission uploaded");
                }
                finishUpload(type);
            }
            return;
        }

        if (mMode == MODE_DOWNLOAD && mLastSent != null
                && mLastSent.getCmd() == MissionPacket.CMD_REQUEST_LIST) {
            if (cmd == MissionPacket.CMD_COUNT) {
                mDownTotal = MissionPacket.parseU16(payload, 1);
                if (mDownTotal <= 0) {
                    finishDownload(MissionPacket.MISSION_ACCEPTED);
                } else {
                    mListener.onTransferStatus("Downloading 0/" + mDownTotal);
                    send(MissionPacket.request(0));
                }
            } else if (cmd == MissionPacket.CMD_ACK) {
                finishDownload(MissionPacket.parseU8(payload, 1));
            }
            return;
        }

        if (mMode == MODE_DOWNLOAD && mLastSent != null
                && mLastSent.getCmd() == MissionPacket.CMD_REQUEST) {
            if (cmd == MissionPacket.CMD_ITEM) {
                PlannerItem item = MissionPacket.parseItem(payload);
                int seq = MissionPacket.parseU16(payload, 1);
                item.frame = PlannerItem.FRAME_GLOBAL_RELATIVE_ALT;
                mDownMission.add(item);
                if (seq + 1 >= mDownTotal) {
                    finishDownload(MissionPacket.MISSION_ACCEPTED);
                } else {
                    mListener.onTransferStatus("Downloading " + (seq + 1) + "/" + mDownTotal);
                    send(MissionPacket.request(seq + 1));
                }
            } else if (cmd == MissionPacket.CMD_ACK) {
                finishDownload(MissionPacket.parseU8(payload, 1));
            }
            return;
        }

        if (mMode == MODE_SIMPLE && cmd == mSimpleCmd) {
            boolean ok = MissionPacket.parseU8(payload, 1) == 0;
            mHandler.removeCallbacks(mResend);
            mLastSent = null;
            mMode = MODE_IDLE;
            mListener.onTransferStatus(ok ? "Accepted" : "Refused");
            mListener.onCommandFinished(mSimpleCmd, ok);
        }
    }

    private void finishUpload(int result) {
        mHandler.removeCallbacks(mResend);
        mLastSent = null;
        mMode = MODE_IDLE;
        mUpItems = null;
        mListener.onUploadFinished(result);
    }

    private void finishDownload(int result) {
        mHandler.removeCallbacks(mResend);
        mLastSent = null;
        mMode = MODE_IDLE;
        PlannerMission mission = mDownMission;
        mDownMission = null;
        mListener.onDownloadFinished(mission, result);
    }

    private void fail(String message) {
        mListener.onTransferStatus(message);
        if (mMode == MODE_UPLOAD) {
            mListener.onUploadFinished(MissionPacket.MISSION_ERROR);
        } else if (mMode == MODE_DOWNLOAD) {
            mListener.onDownloadFinished(null, MissionPacket.MISSION_ERROR);
        } else if (mMode == MODE_SIMPLE) {
            mListener.onCommandFinished(mSimpleCmd, false);
        }
        mMode = MODE_IDLE;
        mLastSent = null;
    }
}
