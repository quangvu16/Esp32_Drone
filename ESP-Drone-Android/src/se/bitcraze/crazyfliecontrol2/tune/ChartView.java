package se.bitcraze.crazyfliecontrol2.tune;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;
import android.util.AttributeSet;
import android.view.View;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Scrolling strip chart. Every series is scaled on its own, symmetric around zero, so a
 * small angle and a large motor pulse are both readable; the legend gives each series'
 * range. With {@link #setSharedScale} all series use one range instead, so they compare
 * directly (the per-term panels). The zero line is the middle of the plot.
 * Biểu đồ dải cuộn. Mỗi chuỗi được co giãn theo thang riêng, đối xứng quanh mốc không, nên
 * một góc nhỏ và một xung lớn của động cơ đều dễ đọc; chú giải hiển thị khoảng của từng chuỗi.
 * Với {@link #setSharedScale} tất cả chuỗi dùng chung một khoảng để so sánh trực tiếp (các
 * panel theo từng thành phần). Đường không nằm ở giữa đồ thị.
 *
 * <p>Ported from the balance tuner's Kotlin ChartView. Colours are hard-coded ARGB values
 * (a colour-blind tolerant set) so the view does not depend on theme resources.
 * <p>Được port từ ChartView bằng Kotlin của bộ tinh chỉnh balance. Màu là các giá trị ARGB
 * ghi cứng (một bộ màu thân thiện với người mù màu) nên view không phụ thuộc tài nguyên theme.
 */
public class ChartView extends View {

    /** 300 samples at 20 Hz = 15 s of history. */
    /** 300 mẫu ở 20 Hz = 15 s lịch sử. */
    public static final int CAPACITY = 300;

    public static class Series {
        public final String name;
        public final String unit;
        public final int color;
        public final float minRange;
        public boolean enabled = true;

        public Series(String name, String unit, int color, float minRange) {
            this.name = name;
            this.unit = unit;
            this.color = color;
            this.minRange = minRange;
        }
    }

    private List<Series> mSeries = Collections.emptyList();
    private float[][] mData = new float[0][];
    private int mHead;   // index the next sample goes to
    // chỉ số nơi mẫu tiếp theo sẽ được ghi vào
    private int mCount;

    private boolean mPaused;
    private String mTitle;
    private boolean mSharedScale;
    private String mUnit = "";                   // shown once in the legend when sharedScale
    // chỉ hiển thị một lần trong chú giải khi dùng sharedScale
    private float[] mMarkers = new float[0];     // dashed horizontal lines, e.g. saturation
    // các đường ngang nét đứt, ví dụ mức bão hòa

    private final float mDensity = getResources().getDisplayMetrics().density;
    private final Paint mBgPaint = new Paint();
    private final Paint mGridPaint = new Paint();
    private final Paint mLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mMutedPaint;
    private final Paint mDashedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path mPath = new Path();

    public ChartView(Context context) {
        this(context, null);
    }

    public ChartView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mBgPaint.setColor(0xFF101418);
        mGridPaint.setColor(0xFF2E3740);
        mGridPaint.setStrokeWidth(mDensity);
        mLinePaint.setStyle(Paint.Style.STROKE);
        mLinePaint.setStrokeWidth(1.6f * mDensity);
        mLinePaint.setStrokeJoin(Paint.Join.ROUND);
        mTextPaint.setTextSize(11 * mDensity);
        mTextPaint.setTypeface(Typeface.MONOSPACE);
        mMutedPaint = new Paint(mTextPaint);
        mMutedPaint.setColor(0xFF8A929A);
        mDashedPaint.setColor(0xFFF44336);
        mDashedPaint.setStrokeWidth(mDensity);
        mDashedPaint.setPathEffect(new DashPathEffect(new float[] {4 * mDensity, 4 * mDensity}, 0f));
    }

    public List<Series> getSeries() {
        return mSeries;
    }

    /** Replaces the series definitions and resets the buffers. */
    /** Thay các định nghĩa chuỗi và đặt lại các bộ đệm. */
    public void setSeries(List<Series> series) {
        mSeries = series;
        mData = new float[series.size()][CAPACITY];
        mCount = 0;
        mHead = 0;
        invalidate();
    }

    public void add(float[] values) {
        if (mPaused || mData.length == 0) {
            return;
        }
        for (int i = 0; i < mData.length; i++) {
            mData[i][mHead] = values[i];
        }
        mHead = (mHead + 1) % CAPACITY;
        if (mCount < CAPACITY) {
            mCount++;
        }
        invalidate();
    }

    public void clear() {
        mCount = 0;
        mHead = 0;
        invalidate();
    }

    public void setPaused(boolean paused) {
        mPaused = paused;
        invalidate();
    }

    public boolean isPaused() {
        return mPaused;
    }

    public void setTitle(String title) {
        mTitle = title;
        invalidate();
    }

    public void setSharedScale(boolean sharedScale) {
        mSharedScale = sharedScale;
        invalidate();
    }

    public void setUnit(String unit) {
        mUnit = unit != null ? unit : "";
        invalidate();
    }

    public void setMarkers(float[] markers) {
        mMarkers = markers != null ? markers : new float[0];
        invalidate();
    }

    /**
     * Copies the visible window of one series into {@code out}, oldest sample first.
     * Used by the statistics panel; the chart stays the single owner of the buffer.
     * Sao chép cửa sổ hiển thị của một chuỗi vào {@code out}, mẫu cũ nhất đứng trước.
     * Dùng cho bảng thống kê; biểu đồ vẫn là chủ sở hữu duy nhất của bộ đệm.
     *
     * @return the number of samples copied
     * @return số mẫu đã sao chép
     */
    public int copySamples(int seriesIndex, float[] out) {
        if (seriesIndex < 0 || seriesIndex >= mData.length) {
            return 0;
        }
        int n = Math.min(mCount, out.length);
        for (int k = 0; k < n; k++) {
            out[k] = mData[seriesIndex][(mHead - mCount + k + CAPACITY) % CAPACITY];
        }
        return n;
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        int w = MeasureSpec.getSize(widthMeasureSpec);
        setMeasuredDimension(w, resolveSize((int) (240 * mDensity), heightMeasureSpec));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        float w = getWidth();
        float h = getHeight();
        canvas.drawRect(0f, 0f, w, h, mBgPaint);
        float top = (mSharedScale ? 20 : 34) * mDensity;
        float bottom = h - 6 * mDensity;
        float mid = (top + bottom) / 2f;
        float half = (bottom - top) / 2f;
        for (int k = -2; k <= 2; k++) {
            float y = mid + k * half / 2f;
            mGridPaint.setAlpha(k == 0 ? 255 : 90);
            canvas.drawLine(0f, y, w, y, mGridPaint);
        }
        // 1 s ticks at 20 Hz
        // vạch chia mỗi 1 s ở 20 Hz
        mGridPaint.setAlpha(60);
        for (int s = 0; s < CAPACITY; s += 20) {
            float x = w - s * w / (CAPACITY - 1);
            canvas.drawLine(x, top, x, bottom, mGridPaint);
        }

        float lx = 8 * mDensity;
        float ly = 14 * mDensity;
        float shared = 0f;
        if (mSharedScale) {
            for (int i = 0; i < mSeries.size(); i++) {
                Series ser = mSeries.get(i);
                if (!ser.enabled) {
                    continue;
                }
                shared = Math.max(shared, ser.minRange);
                for (int k = 0; k < mCount; k++) {
                    shared = Math.max(shared, Math.abs(at(i, k)));
                }
            }
            for (float m : mMarkers) {
                if (Math.abs(m) <= shared) {
                    float y = mid - m / shared * half;
                    canvas.drawLine(0f, y, w, y, mDashedPaint);
                }
            }
            String head = (mTitle != null ? mTitle + " " : "") + "±" + fmt(shared) + mUnit;
            canvas.drawText(head, lx, ly, mMutedPaint);
            lx += mMutedPaint.measureText(head) + 12 * mDensity;
        }
        for (int i = 0; i < mSeries.size(); i++) {
            Series ser = mSeries.get(i);
            if (!ser.enabled) {
                continue;
            }
            float range = ser.minRange;
            if (mSharedScale) {
                range = shared;
            } else {
                for (int k = 0; k < mCount; k++) {
                    range = Math.max(range, Math.abs(at(i, k)));
                }
            }
            if (mCount > 1 && range > 0f) {
                mPath.reset();
                for (int k = 0; k < mCount; k++) {
                    // newest sample at the right edge
                    // mẫu mới nhất ở mép phải
                    float x = w - (mCount - 1 - k) * w / (CAPACITY - 1);
                    float y = mid - at(i, k) / range * half;
                    if (k == 0) {
                        mPath.moveTo(x, y);
                    } else {
                        mPath.lineTo(x, y);
                    }
                }
                mLinePaint.setColor(ser.color);
                canvas.drawPath(mPath, mLinePaint);
            }
            mTextPaint.setColor(ser.color);
            String label = mSharedScale ? ser.name : ser.name + " ±" + fmt(range) + ser.unit;
            float tw = mTextPaint.measureText(label) + 12 * mDensity;
            if (lx + tw > w) {
                lx = 8 * mDensity;
                ly += 14 * mDensity;
            }
            canvas.drawText(label, lx, ly, mTextPaint);
            lx += tw;
        }
        if (mCount == 0) {
            String msg = "Waiting for telemetry";
            canvas.drawText(msg, (w - mMutedPaint.measureText(msg)) / 2f, mid - 6 * mDensity, mMutedPaint);
        }
        if (mPaused) {
            canvas.drawText("PAUSED", w - mMutedPaint.measureText("PAUSED") - 8 * mDensity, bottom - 6 * mDensity, mMutedPaint);
        }
    }

    /** k-th oldest stored sample of series i. */
    /** Mẫu đã lưu cũ thứ k của chuỗi i. */
    private float at(int i, int k) {
        return mData[i][(mHead - mCount + k + CAPACITY) % CAPACITY];
    }

    private static String fmt(float v) {
        if (v >= 100) {
            return String.format(Locale.US, "%.0f", v);
        }
        if (v >= 10) {
            return String.format(Locale.US, "%.1f", v);
        }
        return String.format(Locale.US, "%.2f", v);
    }
}
