package se.bitcraze.crazyfliecontrol2.tune;

import java.util.Random;

/**
 * Deep ensemble (Lakshminarayanan et al. 2017) of small neural networks, d -&gt; 32 -&gt; 32 -&gt; 1
 * (tanh hidden layers, linear output), each from its own random start, trained full-batch with
 * Adam and weight decay in plain Java (port of ml_tune.py's MlpEnsemble, no dependencies).
 * Tập mô hình sâu (Lakshminarayanan và cộng sự, 2017) gồm các mạng nơ-ron nhỏ, d -&gt; 32 -&gt; 32 -&gt; 1
 * (các lớp ẩn tanh, đầu ra tuyến tính), mỗi mạng khởi tạo ngẫu nhiên riêng, huấn luyện full-batch
 * với Adam và weight decay bằng Java thuần (port từ MlpEnsemble của ml_tune.py, không phụ thuộc).
 *
 * <p>Targets are standardised to zero mean / unit sd before training and de-standardised after.
 * The spread between the members (population std) stands in for the GP's uncertainty — it is
 * what the neural tuner's lower confidence bound balances against the mean.
 * <p>Các mục tiêu được chuẩn hóa về trung bình không / độ lệch chuẩn đơn vị trước khi huấn luyện
 * và được giải chuẩn hóa sau đó. Độ phân tán giữa các thành viên (population std) thay cho sự
 * bất định của GP — đó là thứ mà lower confidence bound của bộ tinh chỉnh nơ-ron cân bằng
 * với trung bình.
 */
public class MlpEnsemble {

    private static final int MEMBERS = 5;
    private static final int HIDDEN = 32;
    private static final int STEPS = 500;
    private static final double LR = 0.01;
    private static final double DECAY = 1e-3;
    private static final double BETA1 = 0.9;
    private static final double BETA2 = 0.999;
    private static final double ADAM_EPS = 1e-8;

    private final double mMu;
    private final double mSd;
    /** One net = the 6 parameter matrices W0, b0, W1, b1, W2, b2 of ml_tune.py's train(). */
    /** Một mạng = 6 ma trận tham số W0, b0, W1, b1, W2, b2 trong hàm train() của ml_tune.py. */
    private final double[][][][] mNets;

    public MlpEnsemble(double[][] x, double[] y, Random rng) {
        int n = y.length;
        double mu = 0;
        for (double v : y) {
            mu += v;
        }
        mu /= n;
        double sdev = 0;
        for (double v : y) {
            sdev += (v - mu) * (v - mu);
        }
        sdev = Math.sqrt(sdev / n);
        mMu = mu;
        mSd = sdev > 1e-9 ? sdev : 1.0;
        double[] z = new double[n];
        for (int i = 0; i < n; i++) {
            z[i] = (y[i] - mu) / mSd;
        }
        mNets = new double[MEMBERS][][][];
        for (int m = 0; m < MEMBERS; m++) {
            mNets[m] = train(x, z, rng);
        }
    }

    /**
     * Predict {mean, std} over the rows of {@code xs}: the ensemble mean de-standardised, and
     * the spread across members (population std) de-scaled by the target sd.
     * Dự đoán {mean, std} trên các dòng của {@code xs}: trung bình của tập mô hình được giải
     * chuẩn hóa, và độ phân tán giữa các thành viên (population std) được đưa về thang gốc theo
     * độ lệch chuẩn của mục tiêu.
     */
    public double[][] predict(double[][] xs) {
        int n = xs.length;
        double[][] outs = new double[MEMBERS][];
        for (int m = 0; m < MEMBERS; m++) {
            outs[m] = forward(mNets[m], xs);
        }
        double[] mean = new double[n];
        double[] std = new double[n];
        for (int i = 0; i < n; i++) {
            double mm = 0;
            for (int m = 0; m < MEMBERS; m++) {
                mm += outs[m][i];
            }
            mm /= MEMBERS;
            double s = 0;
            for (int m = 0; m < MEMBERS; m++) {
                double d = outs[m][i] - mm;
                s += d * d;
            }
            mean[i] = mMu + mSd * mm;
            std[i] = mSd * Math.sqrt(s / MEMBERS);
        }
        return new double[][] {mean, std};
    }

    /** Forward pass of one net over the rows of x: tanh(tanh(x W0 + b0) W1 + b1) W2 + b2. */
    /** Lan truyền xuôi của một mạng trên các dòng của x: tanh(tanh(x W0 + b0) W1 + b1) W2 + b2. */
    private static double[] forward(double[][][] w, double[][] x) {
        double[][] h1 = tanh(addBias(matmul(x, w[0]), w[1]));
        double[][] h2 = tanh(addBias(matmul(h1, w[2]), w[3]));
        double[][] out = addBias(matmul(h2, w[4]), w[5]);
        double[] y = new double[x.length];
        for (int i = 0; i < y.length; i++) {
            y[i] = out[i][0];
        }
        return y;
    }

    /** One member: 500 full-batch Adam steps with weight decay, from its own random start. */
    /** Một thành viên: 500 bước Adam full-batch với weight decay, khởi tạo ngẫu nhiên riêng. */
    private static double[][][] train(double[][] x, double[] z, Random rng) {
        int n = x.length;
        int d = x[0].length;
        int h = HIDDEN;
        double[][][] w = {
                randomMatrix(d, h, 1 / Math.sqrt(d), rng), new double[1][h],
                randomMatrix(h, h, 1 / Math.sqrt(h), rng), new double[1][h],
                randomMatrix(h, 1, 1 / Math.sqrt(h), rng), new double[1][1],
        };
        double[][][] m1 = {new double[d][h], new double[1][h], new double[h][h],
                new double[1][h], new double[h][1], new double[1][1]};
        double[][][] m2 = {new double[d][h], new double[1][h], new double[h][h],
                new double[1][h], new double[h][1], new double[1][1]};
        for (int t = 1; t <= STEPS; t++) {
            // forward
            // lan truyền xuôi
            double[][] h1 = tanh(addBias(matmul(x, w[0]), w[1]));
            double[][] h2 = tanh(addBias(matmul(h1, w[2]), w[3]));
            double[][] out = addBias(matmul(h2, w[4]), w[5]);
            // backward: g_out = 2 * (out - z) / n, then the chain rule per layer
            // ngược: g_out = 2 * (out - z) / n, sau đó quy tắc chuỗi cho từng lớp
            double[][] gOut = new double[n][1];
            for (int i = 0; i < n; i++) {
                gOut[i][0] = 2 * (out[i][0] - z[i]) / n;
            }
            double[][][] g = new double[6][][];
            g[4] = matmul(transpose(h2), gOut);                    // h x 1
            g[5] = colSums(gOut);                                  // 1 x 1
            double[][] gH2 = hadamard(matmul(gOut, transpose(w[4])), oneMinusSquare(h2));
            g[2] = matmul(transpose(h1), gH2);                     // h x h
            g[3] = colSums(gH2);                                   // 1 x h
            double[][] gH1 = hadamard(matmul(gH2, transpose(w[2])), oneMinusSquare(h1));
            g[0] = matmul(transpose(x), gH1);                      // d x h
            g[1] = colSums(gH1);                                   // 1 x h
            // Adam step; weight decay rides on the gradient, biases included
            // bước Adam; weight decay được cộng vào gradient, kể cả bias
            double c1 = 1 - Math.pow(BETA1, t);
            double c2 = 1 - Math.pow(BETA2, t);
            for (int j = 0; j < 6; j++) {
                for (int r = 0; r < w[j].length; r++) {
                    for (int c = 0; c < w[j][r].length; c++) {
                        double gj = g[j][r][c] + DECAY * w[j][r][c];
                        m1[j][r][c] = BETA1 * m1[j][r][c] + (1 - BETA1) * gj;
                        m2[j][r][c] = BETA2 * m2[j][r][c] + (1 - BETA2) * gj * gj;
                        w[j][r][c] -= LR * (m1[j][r][c] / c1)
                                / (Math.sqrt(m2[j][r][c] / c2) + ADAM_EPS);
                    }
                }
            }
        }
        return w;
    }

    // ------------------------------------------------------------- dense-matrix helpers

    private static double[][] randomMatrix(int rows, int cols, double sd, Random rng) {
        double[][] out = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                out[r][c] = sd * rng.nextGaussian();
            }
        }
        return out;
    }

    private static double[][] matmul(double[][] a, double[][] b) {
        int n = a.length;
        int k = b.length;
        int m = b[0].length;
        double[][] out = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int p = 0; p < k; p++) {
                double aip = a[i][p];
                if (aip == 0) {
                    continue;
                }
                for (int j = 0; j < m; j++) {
                    out[i][j] += aip * b[p][j];
                }
            }
        }
        return out;
    }

    private static double[][] transpose(double[][] a) {
        double[][] out = new double[a[0].length][a.length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[j][i] = a[i][j];
            }
        }
        return out;
    }

    private static double[][] tanh(double[][] a) {
        double[][] out = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[i][j] = Math.tanh(a[i][j]);
            }
        }
        return out;
    }

    /** Row-wise + a 1 x m bias row (numpy broadcasting of the bias vectors). */
    /** Cộng theo từng dòng với một hàng bias 1 x m (numpy broadcasting của các vector bias). */
    private static double[][] addBias(double[][] a, double[][] bias) {
        double[][] out = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[i][j] = a[i][j] + bias[0][j];
            }
        }
        return out;
    }

    private static double[][] hadamard(double[][] a, double[][] b) {
        double[][] out = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[i][j] = a[i][j] * b[i][j];
            }
        }
        return out;
    }

    private static double[][] oneMinusSquare(double[][] a) {
        double[][] out = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[i][j] = 1 - a[i][j] * a[i][j];
            }
        }
        return out;
    }

    /** Column sums as a 1 x m matrix (the bias gradients). */
    /** Tổng theo cột dưới dạng ma trận 1 x m (các gradient của bias). */
    private static double[][] colSums(double[][] a) {
        double[][] out = new double[1][a[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < a[0].length; j++) {
                out[0][j] += a[i][j];
            }
        }
        return out;
    }
}
