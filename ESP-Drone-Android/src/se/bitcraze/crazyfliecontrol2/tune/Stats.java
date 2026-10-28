package se.bitcraze.crazyfliecontrol2.tune;

/**
 * Min/max/mean/stddev/RMS over a window of samples, for the statistics panel.
 * Min/max/mean/stddev/RMS trên một cửa sổ mẫu, dùng cho bảng thống kê.
 */
public final class Stats {

    public final int n;
    public final float min;
    public final float max;
    public final double mean;
    public final double stdDev;
    public final double rms;

    private Stats(int n, float min, float max, double mean, double stdDev, double rms) {
        this.n = n;
        this.min = min;
        this.max = max;
        this.mean = mean;
        this.stdDev = stdDev;
        this.rms = rms;
    }

    /** @param samples window data, oldest first; only the first {@code n} entries are used. */
    /** @param samples dữ liệu cửa sổ, cũ nhất đứng trước; chỉ {@code n} phần tử đầu tiên được dùng. */
    public static Stats of(float[] samples, int n) {
        if (n <= 0) {
            return new Stats(0, 0f, 0f, 0, 0, 0);
        }
        float min = samples[0];
        float max = samples[0];
        double sum = 0;
        double sq = 0;
        for (int i = 0; i < n; i++) {
            float v = samples[i];
            min = Math.min(min, v);
            max = Math.max(max, v);
            sum += v;
            sq += v * v;
        }
        double mean = sum / n;
        double var = 0;
        for (int i = 0; i < n; i++) {
            double d = samples[i] - mean;
            var += d * d;
        }
        return new Stats(n, min, max, mean, Math.sqrt(var / n), Math.sqrt(sq / n));
    }

    /** Fixed-capacity ring buffer for derived series (rate, motor) the charts do not hold. */
    /** Bộ đệm vòng có dung lượng cố định cho các chuỗi dẫn xuất (rate, motor) mà biểu đồ không lưu. */
    public static final class Ring {
        private final float[] mData;
        private int mHead;
        private int mCount;

        public Ring(int capacity) {
            mData = new float[capacity];
        }

        public synchronized void add(float value) {
            mData[mHead] = value;
            mHead = (mHead + 1) % mData.length;
            if (mCount < mData.length) {
                mCount++;
            }
        }

        public synchronized void clear() {
            mCount = 0;
            mHead = 0;
        }

        /** Oldest-first copy of the whole buffer; returns the number of samples written. */
        /** Bản sao toàn bộ bộ đệm theo thứ tự cũ nhất trước; trả về số mẫu đã ghi. */
        public synchronized int copyTo(float[] out) {
            int n = Math.min(mCount, out.length);
            for (int k = 0; k < n; k++) {
                out[k] = mData[(mHead - mCount + k + mData.length) % mData.length];
            }
            return n;
        }
    }
}
