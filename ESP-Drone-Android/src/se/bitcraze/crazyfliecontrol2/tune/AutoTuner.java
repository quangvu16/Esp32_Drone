package se.bitcraze.crazyfliecontrol2.tune;

import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

import se.bitcraze.crazyflie.lib.param.Param;

/**
 * Auto-tune of one axis' rate-PID gains while the drone sits on a stand with the props
 * spinning. Port of the balance tuner's AutoTuner trial framework with exactly four
 * optimizers:
 *
 * <ul>
 *   <li>{@link Method#ML} - Bayesian optimisation: a Gaussian process over the tried
 *       gains learns log(1 + cost), the next trial maximises expected improvement.</li>
 *   <li>{@link Method#TWIDDLE} - coordinate descent: +-25 % on one gain at a time
 *       (x1.3 after a win, x0.6 otherwise, below 4 % = converged), wins re-measured so
 *       noise cannot win.</li>
 *   <li>{@link Method#NELDER_MEAD} - downhill simplex over the unit cube, best vertex
 *       re-measured on a shrink.</li>
 *   <li>{@link Method#RANDOM} - Latin-hypercube samples: the baseline the others must
 *       beat.</li>
 * </ul>
 *
 * Every trial waits for the drone to be level on its stand, applies the candidate gains,
 * settles, measures a window at 20 Hz and scores it with {@link #costOf}. A trial that
 * tips the drone over is "bad": the best gains are put back immediately. Everything the
 * tuner sends is idempotent ({@link Param#setValue} re-applies and re-acks the same
 * value), so the client's 500 ms resend queue is safe.
 *
 * <p>Runs on its own thread; {@link Listener} callbacks are posted to the main thread and
 * telemetry is fed in from the activity with {@link #onTelemetry}.
 */
public class AutoTuner {

    private static final String LOG_TAG = "Crazyflie-AutoTuner";

    private static final double COST_CAP = 300.0;
    /** Tip-over threshold for the drone on its stand, in degrees of roll or pitch. */
    private static final float TIP_DEG = 30f;
    /** Upright window for {@link #waitUpright}: within 15 deg for 1.5 s. */
    private static final float UPRIGHT_DEG = 15f;

    /** The four optimizers; the hints are shown under the method picker. */
    public enum Method {
        ML("ML (Bayesian)", 2.0, "ml",
                "Gaussian process + expected improvement. Fewest trials; the default."),
        TWIDDLE("Twiddle", 3.0, "twiddle",
                "One gain at a time, wins re-measured. Simple, stays close to the start."),
        NELDER_MEAD("Nelder-Mead", 2.0, "nelder-mead",
                "Downhill simplex: local, no model, sensitive to noise."),
        RANDOM("Random (baseline)", 2.0, "random",
                "Latin-hypercube samples: what the others must beat.");

        public final String label;
        /** Search box half-width as a factor on the starting gains. */
        public final double defaultRange;
        public final String tag;
        public final String hint;

        Method(String label, double defaultRange, String tag, String hint) {
            this.label = label;
            this.defaultRange = defaultRange;
            this.tag = tag;
            this.hint = hint;
        }
    }

    /**
     * @param ids    complete names of the three gains of the active axis
     *               (e.g. pid.roll_kp, pid.roll_ki, pid.roll_kd)
     * @param nudge  kept for structural parity with the reference; the drone has no wheels
     *               to nudge, so trials measure the drone as it sits on its stand
     * @param trimAngle0 kept for parity; a quad has no balance-angle trim
     * @param learn  kept for parity; cross-run history seeding is not ported (the trials
     *               of the current run already feed the model)
     */
    public static class Config {
        public final Method method;
        public final List<String> ids;
        public final int trials;
        public final double trialS;
        public final int nudge;
        public final double range;
        public final boolean trimAngle0;
        public final boolean learn;

        public Config(Method method, List<String> ids, int trials, double trialS, int nudge,
                      double range, boolean trimAngle0, boolean learn) {
            this.method = method;
            this.ids = ids;
            this.trials = trials;
            this.trialS = trialS;
            this.nudge = nudge;
            this.range = range;
            this.trimAngle0 = trimAngle0;
            this.learn = learn;
        }
    }

    public interface Listener {
        void onStatus(String text);

        /** Trial {@code trialNo} ran with {@code candidates}; live status for the panel. */
        void onTrial(int trialNo, float[] candidates, double cost);

        void onDone(Report report);
    }

    /** The result of a run: best cost, best gains and every trial that was tried. */
    public static class Report {
        public final Method method;
        public final String reason;
        public final int trialsRun;
        public final double bestCost;
        public final float[] bestGains;
        public final List<float[]> history;
        public final List<Double> historyCosts;

        Report(Method method, String reason, int trialsRun, double bestCost, float[] bestGains,
               List<float[]> history, List<Double> historyCosts) {
            this.method = method;
            this.reason = reason;
            this.trialsRun = trialsRun;
            this.bestCost = bestCost;
            this.bestGains = bestGains;
            this.history = history;
            this.historyCosts = historyCosts;
        }
    }

    /** One 20 Hz sample, fed by the activity from the axis tune log block. */
    public static class Telemetry {
        /** The axis' stabilizer.* value (deg for roll/pitch; yaw is logged as a rate). */
        public final float angle;
        /** Gyro rate in deg/s. See {@link #costOf}: derived for roll/pitch, direct for yaw. */
        public final float rate;
        /** The axis' controller.* output. */
        public final float control;
        /** motor.m1, 1000..2000 us convention. */
        public final float motorM1;
        /** Level angles from the status block, for the upright / tip-over checks. */
        public final float roll;
        public final float pitch;

        public Telemetry(float angle, float rate, float control, float motorM1, float roll, float pitch) {
            this.angle = angle;
            this.rate = rate;
            this.control = control;
            this.motorM1 = motorM1;
            this.roll = roll;
            this.pitch = pitch;
        }
    }

    /** What one trial window measured. */
    public static class Metrics {
        public final double angleMean;
        public final double angleStd;
        public final double rateRms;
        public final double motorRms;
        public final double controlEffort;
        public final double saturatedShare;
        public final double cost;

        Metrics(double angleMean, double angleStd, double rateRms, double motorRms,
                double controlEffort, double saturatedShare, double cost) {
            this.angleMean = angleMean;
            this.angleStd = angleStd;
            this.rateRms = rateRms;
            this.motorRms = motorRms;
            this.controlEffort = controlEffort;
            this.saturatedShare = saturatedShare;
            this.cost = cost;
        }
    }

    /**
     * One trial's cost, adapted from the balance tuner's {@code measure()}:
     *
     * <pre>
     * cost = angleStdDev + 0.1 * rateRms + 0.2 * motorRms + 0.005 * controlEffort
     *      + 20.0 * saturatedShare
     * </pre>
     *
     * <p>Deliberate adaptations (not bugs):
     * <ul>
     *   <li>{@code rateRms}: the drone logs no roll/pitch gyro rates (only stabilizer.yaw
     *       is a rate), so roll/pitch rates are finite-differenced from consecutive
     *       stabilizer.* angle samples at the 20 Hz sample rate; stabilizer.yaw is used
     *       directly as the yaw rate.</li>
     *   <li>{@code motorRms}: RMS of (motor.m1 - 1500), the 1000..2000 us convention, in
     *       place of the robot's wheel-speed RMS.</li>
     *   <li>{@code saturatedShare}: fraction of samples where motor.m1 sits on its rail
     *       (&gt;= 1990 or &lt;= 1010).</li>
     *   <li>The wheel-encoder travel term and the fall cost of the reference are dropped
     *       entirely.</li>
     * </ul>
     */
    public static Metrics costOf(float[] angle, float[] rate, float[] motorM1, float[] control, int n) {
        if (n <= 0) {
            return new Metrics(0, 0, 0, 0, 0, 0, 0);
        }
        double mean = 0;
        for (int i = 0; i < n; i++) {
            mean += angle[i];
        }
        mean /= n;
        double var = 0;
        double rateSq = 0;
        double motorSq = 0;
        double ctrlAbs = 0;
        int sat = 0;
        for (int i = 0; i < n; i++) {
            double d = angle[i] - mean;
            var += d * d;
            rateSq += rate[i] * rate[i];
            double m = motorM1[i] - 1500.0;
            motorSq += m * m;
            ctrlAbs += Math.abs(control[i]);
            if (motorM1[i] >= 1990f || motorM1[i] <= 1010f) {
                sat++;
            }
        }
        double angleStd = Math.sqrt(var / n);
        double rateRms = Math.sqrt(rateSq / n);
        double motorRms = Math.sqrt(motorSq / n);
        double controlEffort = ctrlAbs / n;
        double saturatedShare = (double) sat / n;
        double cost = angleStd + 0.1 * rateRms + 0.2 * motorRms + 0.005 * controlEffort + 20.0 * saturatedShare;
        return new Metrics(mean, angleStd, rateRms, motorRms, controlEffort, saturatedShare, cost);
    }

    /** GP target: log(1 + cost), capped so one disaster trial cannot blow up the model. */
    public static double toY(double cost) {
        return Math.log1p(Math.min(cost, COST_CAP));
    }

    // Unchecked on purpose: the searches are endless loops that unwind through these,
    // exactly like the Kotlin original, and run() turns them into a report.
    private static class Stopped extends RuntimeException {
    }

    private static class TuneError extends RuntimeException {
        TuneError(String msg) {
            super(msg);
        }
    }

    private static class Budget extends RuntimeException {
    }

    private final Config mCfg;
    private final Param mParam;
    private final Listener mListener;
    private final Handler mMain = new Handler(Looper.getMainLooper());
    private final Random mRng = new Random();

    private final Object mTelemLock = new Object();
    private final ArrayList<Long> mTelemTime = new ArrayList<>();
    private final ArrayList<Telemetry> mTelem = new ArrayList<>();
    private final Map<String, Float> mEcho = new HashMap<>();

    private volatile boolean mStopped;
    private Thread mThread;

    private float[] mParams;   // current applied values, aligned with cfg.ids
    private float[] mTried;    // what the last trial really ran with
    private float[] mBest;
    private double mBestCost = Double.POSITIVE_INFINITY;

    private int mK;                                // trials completed
    private boolean mLastBad;                      // the last trial tipped the drone
    private Space mSpace;
    private String mStage;
    private final List<double[]> mXs = new ArrayList<>();
    private final List<Double> mYs = new ArrayList<>();
    private final List<Boolean> mOwn = new ArrayList<>();
    private final List<Boolean> mBadMask = new ArrayList<>();
    private GaussianProcess mGp;
    private double[] mBestX;

    public AutoTuner(Config cfg, Param param, Listener listener) {
        mCfg = cfg;
        mParam = param;
        mListener = listener;
    }

    public boolean isRunning() {
        return mThread != null && mThread.isAlive();
    }

    public void start() {
        mThread = new Thread(new Runnable() {
            @Override
            public void run() {
                AutoTuner.this.run();
            }
        }, "autotune");
        mThread.start();
    }

    /** Takes effect after the current trial; for an immediate halt use Panic Stop. */
    public void stop() {
        mStopped = true;
    }

    public void onTelemetry(Telemetry t) {
        long now = SystemClock.elapsedRealtime();
        synchronized (mTelemLock) {
            mTelemTime.add(now);
            mTelem.add(t);
            while (mTelem.size() > 1500) {
                mTelem.remove(0);
                mTelemTime.remove(0);
            }
        }
    }

    /** Echo of a parameter write/read, dispatched from the activity's ParamListener. */
    public void onParam(String completeName, float value) {
        synchronized (mEcho) {
            mEcho.put(completeName, value);
        }
    }

    // ------------------------------------------------------------- plumbing

    private long now() {
        return SystemClock.elapsedRealtime();
    }

    private List<Telemetry> since(long t0) {
        synchronized (mTelemLock) {
            List<Telemetry> out = new ArrayList<>();
            for (int i = 0; i < mTelemTime.size(); i++) {
                if (mTelemTime.get(i) >= t0) {
                    out.add(mTelem.get(i));
                }
            }
            return out;
        }
    }

    private void checkStop() {
        if (mStopped) {
            throw new Stopped();
        }
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new Stopped();
        }
    }

    private void status(final String s) {
        mMain.post(new Runnable() {
            @Override
            public void run() {
                mListener.onStatus(s);
            }
        });
    }

    /**
     * Write one parameter and confirm it, 3 x 500 ms echo retries (the reference's
     * pattern): the write reply lands in {@link #mEcho} via {@link #onParam} and in the
     * Param's value map, and both are polled before the write is repeated.
     */
    private float setParam(String name, float value) {
        for (int retry = 0; retry < 3; retry++) {
            synchronized (mEcho) {
                mEcho.remove(name);
            }
            mParam.setValue(name, value);
            long end = now() + 500;
            while (now() < end) {
                synchronized (mEcho) {
                    Float echo = mEcho.get(name);
                    if (echo != null) {
                        return echo;
                    }
                }
                try {
                    Number v = mParam.getValue(name);
                    if (v != null && Math.abs(v.doubleValue() - value) <= Math.max(1e-4, Math.abs(value) * 1e-4)) {
                        return v.floatValue();
                    }
                } catch (RuntimeException e) {
                    // the value map can be empty mid-connect; the retry is the answer
                }
                sleep(10);
            }
        }
        throw new TuneError("no echo for " + name);
    }

    private void apply(float[] p) {
        for (int j = 0; j < p.length; j++) {
            if (Math.abs(mParams[j] - p[j]) > 1e-6f) {
                mParams[j] = setParam(mCfg.ids.get(j), p[j]);
            }
        }
    }

    /** Read the starting gains from the drone (read requests + echoes, 1.5 s). */
    private float[] readStartValues() {
        synchronized (mEcho) {
            mEcho.clear();
        }
        for (String id : mCfg.ids) {
            mParam.requestParamUpdate(id);
        }
        float[] out = new float[mCfg.ids.size()];
        long end = now() + 1500;
        while (now() < end) {
            boolean all = true;
            for (int j = 0; j < out.length; j++) {
                Float echo;
                synchronized (mEcho) {
                    echo = mEcho.get(mCfg.ids.get(j));
                }
                if (echo != null) {
                    out[j] = echo;
                } else {
                    try {
                        Number v = mParam.getValue(mCfg.ids.get(j));
                        if (v != null) {
                            out[j] = v.floatValue();
                        } else {
                            all = false;
                        }
                    } catch (RuntimeException e) {
                        all = false;
                    }
                }
            }
            if (all) {
                return out;
            }
            sleep(20);
        }
        throw new TuneError("no parameter values from the drone");
    }

    /** Level on the stand (roll and pitch within 15 deg) for 1.5 s. */
    private void waitUpright(boolean prompt) {
        if (prompt) {
            status("Tipped: stand the drone upright, it self-levels once on its stand");
        }
        while (true) {
            checkStop();
            sleep(250);
            List<Telemetry> f = since(now() - 1500);
            if (f.size() >= 25) {
                boolean level = true;
                for (Telemetry t : f) {
                    if (Math.abs(t.roll) > UPRIGHT_DEG || Math.abs(t.pitch) > UPRIGHT_DEG) {
                        level = false;
                        break;
                    }
                }
                if (level) {
                    return;
                }
            }
        }
    }

    private static boolean tipped(Telemetry t) {
        return Math.abs(t.roll) > TIP_DEG || Math.abs(t.pitch) > TIP_DEG;
    }

    /** One trial with the parameters already applied: settle, measure, score. */
    private Metrics measure() {
        sleep(1500);                                      // settle after the change
        checkStop();
        long t0 = now();
        long trialMs = (long) (mCfg.trialS * 1000);
        while (now() - t0 < trialMs) {
            sleep(100);
            for (Telemetry t : since(t0)) {
                if (tipped(t)) {
                    // the trial is over the moment the drone tips on its stand
                    trialMs = 0;
                    break;
                }
            }
            checkStop();
        }
        List<Telemetry> frames = since(t0);
        if (frames.isEmpty()) {
            throw new TuneError("no telemetry received");
        }
        int badAt = -1;
        for (int i = 0; i < frames.size(); i++) {
            if (tipped(frames.get(i))) {
                badAt = i;
                break;
            }
        }
        mLastBad = badAt >= 0;
        List<Telemetry> ok = badAt >= 0 ? frames.subList(0, badAt) : frames;
        if (ok.size() < 5) {
            ok = frames.subList(0, Math.min(5, frames.size()));
        }
        int n = ok.size();
        float[] angle = new float[n];
        float[] rate = new float[n];
        float[] motor = new float[n];
        float[] control = new float[n];
        for (int i = 0; i < n; i++) {
            Telemetry t = ok.get(i);
            angle[i] = t.angle;
            rate[i] = t.rate;
            motor[i] = t.motorM1;
            control[i] = t.control;
        }
        return costOf(angle, rate, motor, control, n);
    }

    /** Wait level with the last parameters, switch to p, measure, record. */
    private TrialResult evaluate(String stage, float[] p) {
        checkStop();
        waitUpright(false);
        apply(p);
        mTried = mParams.clone();
        Metrics m = measure();
        mK++;
        Log.i(LOG_TAG, String.format(Locale.US,
                "trial %d [%s] cost %.3f (angle sd %.3f, rate rms %.3f, motor rms %.3f, ctrl %.3f, sat %.3f)",
                mK, stage, m.cost, m.angleStd, m.rateRms, m.motorRms, m.controlEffort, m.saturatedShare));
        if (mLastBad) {
            apply(mBest);
            waitUpright(true);
            status("Running");
        }
        return new TrialResult(m.cost, mLastBad);
    }

    private static class TrialResult {
        final double cost;
        final boolean bad;

        TrialResult(double cost, boolean bad) {
            this.cost = cost;
            this.bad = bad;
        }
    }

    // ------------------------------------------------------------- model bookkeeping

    /** Search box around the current gains. */
    private void begin(double factor) {
        mSpace = new Space(mCfg.ids, mParams, factor);
        StringBuilder sb = new StringBuilder(mCfg.method.tag);
        for (String n : mSpace.names) {
            sb.append(sb.length() == mCfg.method.tag.length() ? ":" : "+").append(n);
        }
        mStage = sb.toString();
    }

    /** evaluate() plus the model bookkeeping. */
    private TrialResult trial(float[] p, double[] x) {
        TrialResult r = evaluate(mStage, p);
        // the firmware may clamp/round: learn what was really applied
        double[] u = mSpace.toUnit(toDoubles(mTried));
        if (u == null) {
            u = x;
        }
        if (u != null) {
            mXs.add(u);
            mYs.add(toY(r.cost));
            mOwn.add(Boolean.TRUE);
            mBadMask.add(r.bad);
        }
        mGp = mXs.isEmpty() ? null : new GaussianProcess(mXs, toDoubleArray(mYs));
        if (mGp != null) {
            if (mCfg.method != Method.TWIDDLE) {
                chooseBestMl(mGp);
            } else {
                mBestX = mSpace.toUnit(toDoubles(mBest));
            }
        }
        final int trialNo = mK;
        final float[] candidates = mTried.clone();
        final double cost = r.cost;
        mMain.post(new Runnable() {
            @Override
            public void run() {
                mListener.onTrial(trialNo, candidates, cost);
            }
        });
        return r;
    }

    /** The tried set (this run, did not tip) with the lowest predicted cost. */
    private void chooseBestMl(GaussianProcess model) {
        double bestMean = Double.POSITIVE_INFINITY;
        int jBest = -1;
        for (int i = 0; i < mXs.size(); i++) {
            if (mOwn.get(i) && !mBadMask.get(i)) {
                double mean = model.predict(mXs.get(i))[0];
                if (mean < bestMean) {
                    bestMean = mean;
                    jBest = i;
                }
            }
        }
        if (jBest >= 0) {
            mBestX = mXs.get(jBest);
            mBestCost = Math.expm1(mYs.get(jBest));
            double[] x = mBestX;
            for (int q = 0; q < mBest.length; q++) {
                mBest[q] = (float) mSpace.value(q, x[q]);
            }
        }
    }

    // ------------------------------------------------------------- the four searches

    private void searchMl() {
        begin(mCfg.range);
        int d = mCfg.ids.size();
        List<double[]> init = new ArrayList<>();
        init.add(mSpace.toUnit(toDoubles(mParams)));
        init.addAll(latinHypercube(Math.max(0, d + 2 - mXs.size()), d));
        while (mK < mCfg.trials) {
            checkStop();
            double[] x;
            if (mK < init.size()) {
                x = init.get(mK);
            } else {
                GaussianProcess g = mGp;
                double yRef = Double.POSITIVE_INFINITY;
                for (double[] p : mXs) {
                    yRef = Math.min(yRef, g.predict(p)[0]);
                }
                List<double[]> cand = new ArrayList<>(2600);
                for (int i = 0; i < 2000; i++) {
                    cand.add(randomX(d));
                }
                if (mBestX != null) {
                    for (int i = 0; i < 600; i++) {
                        double[] b = new double[d];
                        for (int j = 0; j < d; j++) {
                            b[j] = Math.max(0.0, Math.min(1.0, mBestX[j] + 0.07 * mRng.nextGaussian()));
                        }
                        cand.add(b);
                    }
                }
                double bestEi = Double.NEGATIVE_INFINITY;
                x = cand.get(0);
                for (double[] c : cand) {
                    double ei = g.expectedImprovement(c, yRef);
                    if (ei > bestEi) {
                        bestEi = ei;
                        x = c;
                    }
                }
            }
            float[] p = mBest.clone();
            for (int j = 0; j < d; j++) {
                p[j] = (float) mSpace.value(j, x[j]);
            }
            trial(p, x);
        }
    }

    /** autotune.py's twiddle over the tuned gains: +-step per gain, wins re-measured. */
    private void searchTwiddle() {
        begin(mCfg.range);
        int d = mCfg.ids.size();
        float[] bestP = mBest.clone();
        final double[] lo = new double[d];
        final double[] hi = new double[d];
        for (int j = 0; j < d; j++) {
            ParamSpec spec = ParamSpec.byName(mCfg.ids.get(j));
            lo[j] = Math.max(spec.min, bestP[j] / mCfg.range);
            hi[j] = Math.min(spec.max, bestP[j] * mCfg.range);
        }
        status("Measuring the starting gains (2 trials)");
        double c0 = trial(bestP, null).cost;
        mBestCost = (c0 + trial(bestP, null).cost) / 2;
        double[] step = new double[d];
        for (int j = 0; j < d; j++) {
            step[j] = 0.25;
        }
        while (mK < mCfg.trials && maxOf(step) > 0.04) {
            for (int j = 0; j < d; j++) {
                checkStop();
                boolean improved = false;
                for (int sign : new int[] {1, -1}) {
                    if (mK >= mCfg.trials) {
                        break;
                    }
                    float[] cand = bestP.clone();
                    double v = bestP[j] * (1 + sign * step[j]);
                    cand[j] = (float) Math.max(lo[j], Math.min(hi[j], v));
                    if (Math.abs(cand[j] - bestP[j]) < 1e-9f) {
                        continue;
                    }
                    TrialResult r = trial(cand, null);
                    if (r.bad || r.cost >= mBestCost || mK >= mCfg.trials) {
                        continue;
                    }
                    TrialResult r2 = trial(cand, null);          // confirm: noise must not win
                    if (!r2.bad && (r.cost + r2.cost) / 2 < mBestCost) {
                        bestP = cand;
                        mBestCost = (r.cost + r2.cost) / 2;
                        mBest = bestP.clone();
                        improved = true;
                        status(String.format(Locale.US, "++ %s = %.4g (best cost %.2f)",
                                mCfg.ids.get(j), bestP[j], mBestCost));
                        break;
                    }
                }
                step[j] *= improved ? 1.3 : 0.6;
                if (mK >= mCfg.trials) {
                    break;
                }
            }
        }
        status(mK < mCfg.trials ? "Steps below 4 %: converged" : "Trial budget used");
    }

    /** Nelder-Mead downhill simplex; the best vertex is re-measured on a shrink. */
    private void searchNelderMead() {
        begin(mCfg.range);
        int d = mCfg.ids.size();
        double[] x0 = mSpace.toUnit(toDoubles(mParams));
        if (x0 == null) {
            x0 = new double[d];
            for (int j = 0; j < d; j++) {
                x0[j] = 0.5;
            }
        }
        try {
            List<double[]> pts = new ArrayList<>();
            pts.add(x0.clone());
            for (int j = 0; j < d; j++) {
                double[] p = x0.clone();
                p[j] += p[j] <= 0.75 ? 0.25 : -0.25;
                pts.add(p);
            }
            List<Object[]> s = new ArrayList<>();   // {Double score, double[] x}
            for (double[] p : pts) {
                s.add(new Object[] {f(p), p});
            }
            Comparator<Object[]> byScore = new Comparator<Object[]>() {
                @Override
                public int compare(Object[] a, Object[] b) {
                    return Double.compare((Double) a[0], (Double) b[0]);
                }
            };
            while (true) {
                checkStop();
                Collections.sort(s, byScore);
                double[] c = new double[d];
                for (int j = 0; j < d; j++) {
                    double sum = 0;
                    for (int i = 0; i < s.size() - 1; i++) {
                        sum += ((double[]) s.get(i)[1])[j];
                    }
                    c[j] = sum / (s.size() - 1);
                }
                double[] xw = (double[]) s.get(s.size() - 1)[1];
                double fw = (Double) s.get(s.size() - 1)[0];
                double[] xr = along(c, xw, -1.0);
                double fr = f(xr);
                if (fr < (Double) s.get(0)[0]) {
                    double[] xe = along(c, xw, -2.0);
                    double fe = f(xe);
                    s.set(s.size() - 1, new Object[] {fe < fr ? fe : fr, fe < fr ? xe : xr});
                } else if (fr < (Double) s.get(s.size() - 2)[0]) {
                    s.set(s.size() - 1, new Object[] {fr, xr});
                } else {
                    double[] xc = along(c, fr < fw ? xr : xw, 0.5);
                    double fc = f(xc);
                    if (fc < Math.min(fr, fw)) {
                        s.set(s.size() - 1, new Object[] {fc, xc});
                    } else {                                  // shrink towards the best, re-measured
                        double[] b = (double[]) s.get(0)[1];
                        List<Object[]> next = new ArrayList<>();
                        next.add(new Object[] {((Double) s.get(0)[0] + f(b)) / 2, b});
                        for (int i = 1; i < s.size(); i++) {
                            double[] x = (double[]) s.get(i)[1];
                            double[] y = new double[d];
                            for (int j = 0; j < d; j++) {
                                y[j] = b[j] + 0.5 * (x[j] - b[j]);
                            }
                            next.add(new Object[] {f(y), y});
                        }
                        s = next;
                    }
                }
            }
        } catch (Budget ignored) {
        }
    }

    /** Baseline: Latin-hypercube random search. */
    private void searchRandom() {
        begin(mCfg.range);
        int d = mCfg.ids.size();
        double[] x0 = mSpace.toUnit(toDoubles(mParams));
        if (x0 == null) {
            x0 = new double[d];
        }
        try {
            List<double[]> pop = new ArrayList<>();
            pop.add(x0.clone());
            pop.addAll(latinHypercube(Math.max(0, Math.max(2, mCfg.trials - mK) - 1), d));
            for (double[] x : pop) {
                if (mK >= mCfg.trials) {
                    break;
                }
                f(x);
            }
        } catch (Budget ignored) {
        }
    }

    /** One trial at unit-cube point x; returns its score toY(cost). */
    private double f(double[] x) {
        if (mK >= mCfg.trials) {
            throw new Budget();
        }
        double[] u = clip(x);
        float[] p = mBest.clone();
        for (int j = 0; j < p.length; j++) {
            p[j] = (float) mSpace.value(j, u[j]);
        }
        return toY(trial(p, u).cost);
    }

    private double[] along(double[] c, double[] x, double t) {
        double[] out = new double[c.length];
        for (int i = 0; i < c.length; i++) {
            out[i] = c[i] + t * (x[i] - c[i]);
        }
        return clip(out);
    }

    private double[] clip(double[] x) {
        double[] out = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            out[i] = Math.max(0.0, Math.min(1.0, x[i]));
        }
        return out;
    }

    private double[] randomX(int d) {
        double[] x = new double[d];
        for (int j = 0; j < d; j++) {
            x[j] = mRng.nextDouble();
        }
        return x;
    }

    private static double maxOf(double[] a) {
        double m = a[0];
        for (double v : a) {
            m = Math.max(m, v);
        }
        return m;
    }

    private static double[] toDoubles(float[] f) {
        double[] out = new double[f.length];
        for (int i = 0; i < f.length; i++) {
            out[i] = f[i];
        }
        return out;
    }

    private static double[] toDoubleArray(List<Double> l) {
        double[] out = new double[l.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = l.get(i);
        }
        return out;
    }

    private List<double[]> latinHypercube(int count, int d) {
        if (count <= 0) {
            return Collections.emptyList();
        }
        double[][] cols = new double[d][count];
        for (int j = 0; j < d; j++) {
            List<Integer> perm = new ArrayList<>();
            for (int i = 0; i < count; i++) {
                perm.add(i);
            }
            Collections.shuffle(perm, mRng);
            for (int i = 0; i < count; i++) {
                cols[j][i] = (perm.get(i) + mRng.nextDouble()) / count;
            }
        }
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double[] x = new double[d];
            for (int j = 0; j < d; j++) {
                x[j] = cols[j][i];
            }
            out.add(x);
        }
        return out;
    }

    // ------------------------------------------------------------- the run

    private void run() {
        String reason = "finished";
        try {
            mParams = readStartValues();
            mBest = mParams.clone();
            status("Waiting for the drone to be level on its stand");
            waitUpright(false);
            status("Running");
            switch (mCfg.method) {
                case ML:
                    searchMl();
                    break;
                case TWIDDLE:
                    searchTwiddle();
                    break;
                case NELDER_MEAD:
                    searchNelderMead();
                    break;
                default:
                    searchRandom();
                    break;
            }
        } catch (Stopped e) {
            reason = "stopped";
        } catch (TuneError e) {
            reason = "error: " + e.getMessage();
        } catch (RuntimeException e) {
            reason = "error: " + e;
            Log.e(LOG_TAG, "auto-tune failed", e);
        } finally {
            if (mBest != null) {
                try {
                    apply(mBest);
                } catch (RuntimeException e) {
                    reason += " (could not apply the best values: " + e.getMessage() + ")";
                }
            }
            final Report report = buildReport(reason);
            mMain.post(new Runnable() {
                @Override
                public void run() {
                    mListener.onDone(report);
                }
            });
        }
    }

    private Report buildReport(String reason) {
        List<float[]> history = new ArrayList<>();
        List<Double> historyCosts = new ArrayList<>();
        if (mSpace != null) {
            for (int i = 0; i < mXs.size(); i++) {
                if (mOwn.get(i)) {
                    double[] x = mXs.get(i);
                    float[] g = new float[x.length];
                    for (int q = 0; q < x.length; q++) {
                        g[q] = (float) mSpace.value(q, x[q]);
                    }
                    history.add(g);
                    historyCosts.add(Math.expm1(mYs.get(i)));
                }
            }
        }
        float[] bestGains = mBest != null ? mBest.clone() : new float[0];
        double bestCost = mBestCost == Double.POSITIVE_INFINITY ? Double.NaN : mBestCost;
        return new Report(mCfg.method, reason, mK, bestCost, bestGains, history, historyCosts);
    }

    /**
     * Box around the starting values, mapped to [0,1]^d. The drone's gains are all
     * positive, so the whole box is log-scaled (current / factor .. current x factor,
     * clamped to the parameter's limits); the reference's linear offset parameter has no
     * counterpart here.
     */
    public static class Space {
        public final List<String> ids;
        public final String[] names;
        public final double[] lo;
        public final double[] hi;

        public Space(List<String> ids, float[] centre, double factor) {
            this.ids = ids;
            this.names = ids.toArray(new String[0]);
            lo = new double[ids.size()];
            hi = new double[ids.size()];
            for (int k = 0; k < ids.size(); k++) {
                ParamSpec spec = ParamSpec.byName(ids.get(k));
                double c = centre[k] > 0 ? centre[k] : 0.05 * spec.max;
                lo[k] = Math.log(Math.max(c / factor, 1e-3 * spec.max));
                hi[k] = Math.log(Math.min(c * factor, spec.max));
                if (hi[k] <= lo[k]) {
                    hi[k] = lo[k] + Math.log(2);   // degenerate edge: give the box some width
                }
            }
        }

        public double value(int k, double u) {
            return Math.exp(lo[k] + u * (hi[k] - lo[k]));
        }

        /** null when a value lies outside the box (or a gain is not positive). */
        public double[] toUnit(double[] values) {
            double[] x = new double[values.length];
            for (int k = 0; k < values.length; k++) {
                if (values[k] <= 0) {
                    return null;
                }
                double u = (Math.log(values[k]) - lo[k]) / (hi[k] - lo[k]);
                if (u < -1e-6 || u > 1 + 1e-6) {
                    return null;
                }
                x[k] = Math.max(0.0, Math.min(1.0, u));
            }
            return x;
        }
    }
}
