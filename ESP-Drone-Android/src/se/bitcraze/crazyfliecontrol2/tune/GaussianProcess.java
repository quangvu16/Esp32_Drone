package se.bitcraze.crazyfliecontrol2.tune;

import java.util.List;

/**
 * Zero-mean Gaussian process on standardised targets, RBF kernel shared over unit-cube
 * inputs (port of the balance tuner's GaussianProcess, itself ml_tune.py). The length
 * scale and noise are picked by log marginal likelihood over a small fixed grid; the
 * Cholesky factor is reused for every prediction.
 */
public class GaussianProcess {

    private static final double[] LENGTHS = {0.08, 0.15, 0.25, 0.4, 0.7, 1.2};
    private static final double[] NOISES = {0.01, 0.05, 0.15, 0.4};

    private final List<double[]> mXs;
    private final double mMu;
    private final double mSd;
    private final double mLs;
    private final double[][] mChol;
    private final double[] mAlpha;

    public GaussianProcess(List<double[]> xs, double[] y) {
        mXs = xs;
        double mu = 0;
        for (double v : y) {
            mu += v;
        }
        mu /= y.length;
        mMu = mu;
        double sdev = 0;
        for (double v : y) {
            sdev += (v - mu) * (v - mu);
        }
        sdev = Math.sqrt(sdev / y.length);
        double sd = (y.length > 1 && sdev > 1e-9) ? sdev : 1.0;
        mSd = sd;
        double[] z = new double[y.length];
        for (int i = 0; i < y.length; i++) {
            z[i] = (y[i] - mu) / sd;
        }

        double best = Double.NEGATIVE_INFINITY;
        double bestLs = LENGTHS[0];
        double[][] bestChol = null;
        double[] bestAlpha = null;
        for (double l : LENGTHS) {
            for (double noise : NOISES) {
                int n = xs.size();
                double[][] k = new double[n][n];
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) {
                        k[i][j] = rbf(xs.get(i), xs.get(j), l) + (i == j ? noise : 0.0);
                    }
                }
                double[][] c = cholesky(k);
                if (c == null) {
                    continue;
                }
                double[] a = backSolve(c, forwardSolve(c, z));
                double lml = 0;
                for (int i = 0; i < n; i++) {
                    lml += -0.5 * z[i] * a[i] - Math.log(c[i][i]);
                }
                if (lml > best) {
                    best = lml;
                    bestLs = l;
                    bestChol = c;
                    bestAlpha = a;
                }
            }
        }
        if (bestChol == null) {
            // Degenerate sample set (every grid candidate failed). The Kotlin original
            // crashed on its lateinit fields here; answer with a jittered prior instead.
            int n = xs.size();
            double[][] k = new double[n][n];
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    k[i][j] = rbf(xs.get(i), xs.get(j), LENGTHS[0]) + (i == j ? 1.0 : 0.0);
                }
            }
            bestChol = cholesky(k);
            if (bestChol == null) {
                bestChol = new double[n][n];
                for (int i = 0; i < n; i++) {
                    bestChol[i][i] = 1.0;
                }
                bestAlpha = z.clone();
            } else {
                bestAlpha = backSolve(bestChol, forwardSolve(bestChol, z));
            }
        }
        mLs = bestLs;
        mChol = bestChol;
        mAlpha = bestAlpha;
    }

    /** Posterior {mean, std} of the target at x, in the original (un-standardised) units. */
    public double[] predict(double[] x) {
        double[] ks = new double[mXs.size()];
        for (int i = 0; i < ks.length; i++) {
            ks[i] = rbf(x, mXs.get(i), mLs);
        }
        double mean = 0;
        for (int i = 0; i < ks.length; i++) {
            mean += ks[i] * mAlpha[i];
        }
        double[] v = forwardSolve(mChol, ks);
        double sq = 0;
        for (double d : v) {
            sq += d * d;
        }
        double variance = Math.max(1e-12, 1 - sq);
        return new double[] {mMu + mSd * mean, mSd * Math.sqrt(variance)};
    }

    public double expectedImprovement(double[] x, double yBest) {
        return expectedImprovement(x, yBest, 0.01);
    }

    public double expectedImprovement(double[] x, double yBest, double xi) {
        double[] ms = predict(x);
        double imp = yBest - ms[0] - xi;
        double z = imp / ms[1];
        return imp * normCdf(z) + ms[1] * Math.exp(-0.5 * z * z) / Math.sqrt(2 * Math.PI);
    }

    private static double rbf(double[] a, double[] b, double l) {
        double s = 0;
        for (int i = 0; i < a.length; i++) {
            double d = a[i] - b[i];
            s += d * d;
        }
        return Math.exp(-0.5 * s / (l * l));
    }

    /** Abramowitz-Stegun 7.1.26 erf, |error| < 1.5e-7. */
    public static double normCdf(double z) {
        double x = Math.abs(z) / Math.sqrt(2.0);
        double t = 1 / (1 + 0.3275911 * x);
        double erf = 1 - t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429)))) * Math.exp(-x * x);
        return 0.5 * (1 + (z >= 0 ? erf : -erf));
    }

    private static double[][] cholesky(double[][] a) {
        int n = a.length;
        double[][] l = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double s = a[i][j];
                for (int k = 0; k < j; k++) {
                    s -= l[i][k] * l[j][k];
                }
                if (i == j) {
                    if (s <= 0) {
                        return null;
                    }
                    l[i][i] = Math.sqrt(s);
                } else {
                    l[i][j] = s / l[j][j];
                }
            }
        }
        return l;
    }

    /** L x = b */
    private static double[] forwardSolve(double[][] l, double[] b) {
        double[] x = new double[b.length];
        for (int i = 0; i < b.length; i++) {
            double s = b[i];
            for (int k = 0; k < i; k++) {
                s -= l[i][k] * x[k];
            }
            x[i] = s / l[i][i];
        }
        return x;
    }

    /** L^T x = b */
    private static double[] backSolve(double[][] l, double[] b) {
        double[] x = new double[b.length];
        for (int i = b.length - 1; i >= 0; i--) {
            double s = b[i];
            for (int k = i + 1; k < b.length; k++) {
                s -= l[k][i] * x[k];
            }
            x[i] = s / l[i][i];
        }
        return x;
    }
}
