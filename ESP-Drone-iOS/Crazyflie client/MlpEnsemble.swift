//
//  MlpEnsemble.swift
//  Crazyflie client
//
//  Deep ensemble (Lakshminarayanan et al. 2017) of small neural networks —
//  a direct port of ml_tune.py's `MlpEnsemble`: d -> 32 -> 32 -> 1 with tanh
//  hidden layers and a linear output, each member from its own random start,
//  trained full-batch with Adam and weight decay in plain Swift. The spread
//  between members stands in for the GP's uncertainty (predict's std).
//
//  Targets are standardised to zero mean / unit sd before training and the
//  predictions are de-standardised after; weight decay is applied to the
//  gradient (g += DECAY * W) before the Adam step, as in the reference.
//

import Foundation

/// The fitted cost surrogate the model view draws. Both the Gaussian process
/// (AutoTuner.swift) and this ensemble speak it — the trial payload carries
/// whichever the run fitted, as ml_tune.py's `post("trial", …)` carries `gp`.
protocol CostModel {
    /// Mean and std of y = log1p(min(cost, 300)) at a unit-cube point.
    func predict(_ x: [Double]) -> (mean: Double, std: Double)
}

extension RandomNumberGenerator {
    /// N(0, 1) via Box-Muller — the reference's `rng.normal` / `nextGaussian`.
    mutating func nextGaussian() -> Double {
        let u1 = 1 - Double.random(in: 0..<1, using: &self)   // (0, 1], so log(u1) is finite
        let u2 = Double.random(in: 0..<1, using: &self)
        return (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
    }
}

final class MlpEnsemble: CostModel {
    static let members = 5          // MEMBERS
    static let hidden = 32          // HIDDEN
    static let steps = 500          // STEPS
    static let lr = 0.01            // LR
    static let decay = 1e-3         // DECAY

    private let mu: Double
    private let sd: Double
    private let nets: [Net]

    /// Weights of one member: d -> HIDDEN -> HIDDEN -> 1 (biases as 1-wide
    /// rows so every parameter goes through the same Adam step).
    private struct Net {
        var w0: [[Double]]      // d x HIDDEN
        var b0: [Double]        // HIDDEN
        var w1: [[Double]]      // HIDDEN x HIDDEN
        var b1: [Double]        // HIDDEN
        var w2: [Double]        // HIDDEN
        var b2: [Double]        // 1

        static func zeros(like n: Net) -> Net {
            Net(w0: n.w0.map { $0.map { _ in 0 } },
                b0: n.b0.map { _ in 0 },
                w1: n.w1.map { $0.map { _ in 0 } },
                b1: n.b1.map { _ in 0 },
                w2: n.w2.map { _ in 0 },
                b2: n.b2.map { _ in 0 })
        }
    }

    init<R: RandomNumberGenerator>(xs: [[Double]], y: [Double], rng: inout R) {
        let mean = y.reduce(0, +) / Double(y.count)
        let sdev = (y.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(y.count)).squareRoot()
        let sd = sdev > 1e-9 ? sdev : 1.0
        self.mu = mean
        self.sd = sd
        let z = y.map { ($0 - mean) / sd }
        var nets: [Net] = []
        for _ in 0..<MlpEnsemble.members {
            nets.append(MlpEnsemble.train(xs: xs, z: z, rng: &rng))
        }
        self.nets = nets
    }

    /// Ensemble mean and spread of the cost (in y units) at `x`.
    func predict(_ x: [Double]) -> (mean: Double, std: Double) {
        var out: [Double] = []
        for n in nets {
            out.append(MlpEnsemble.forward(n, x).out)
        }
        let mean = out.reduce(0, +) / Double(out.count)
        let variance = out.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Double(out.count)
        return (mu + sd * mean, sd * variance.squareRoot())
    }

    // MARK: - Private

    /// One member's forward pass; fills the hidden layers for backprop.
    private static func forward(_ n: Net, _ x: [Double]) -> (h1: [Double], h2: [Double], out: Double) {
        let h = hidden
        var h1 = [Double](repeating: 0, count: h)
        for a in 0..<h {
            var s = n.b0[a]
            for i in 0..<x.count {
                s += x[i] * n.w0[i][a]
            }
            h1[a] = tanh(s)
        }
        var h2 = [Double](repeating: 0, count: h)
        var out = n.b2[0]
        for b in 0..<h {
            var s = n.b1[b]
            for a in 0..<h {
                s += h1[a] * n.w1[a][b]
            }
            h2[b] = tanh(s)
            out += h2[b] * n.w2[b]
        }
        return (h1, h2, out)
    }

    /// Full-batch training with Adam — ml_tune.py's `train`, one weight-decayed
    /// gradient over the whole set per step.
    private static func train<R: RandomNumberGenerator>(xs: [[Double]], z: [Double], rng: inout R) -> Net {
        let d = xs[0].count
        let h = hidden
        let n = Double(xs.count)
        // W ~ N(0, 1/sqrt(fan_in)), biases zero.
        var net = Net(
            w0: (0..<d).map { _ in (0..<h).map { _ in rng.nextGaussian() / sqrt(Double(d)) } },
            b0: [Double](repeating: 0, count: h),
            w1: (0..<h).map { _ in (0..<h).map { _ in rng.nextGaussian() / sqrt(Double(h)) } },
            b1: [Double](repeating: 0, count: h),
            w2: (0..<h).map { _ in rng.nextGaussian() / sqrt(Double(h)) },
            b2: [0])
        var m1 = Net.zeros(like: net)
        var m2 = Net.zeros(like: net)

        for t in 1...steps {
            var g = Net.zeros(like: net)
            var g2 = [Double](repeating: 0, count: h)
            for s in xs.indices {
                let (h1, h2, out) = forward(net, xs[s])
                let go = 2 * (out - z[s]) / n
                g.b2[0] += go
                for b in 0..<h {
                    g.w2[b] += h2[b] * go
                    g2[b] = go * net.w2[b] * (1 - h2[b] * h2[b])
                    g.b1[b] += g2[b]
                }
                for a in 0..<h {
                    var back = 0.0
                    for b in 0..<h {
                        g.w1[a][b] += h1[a] * g2[b]
                        back += g2[b] * net.w1[a][b]
                    }
                    let g1 = back * (1 - h1[a] * h1[a])
                    g.b0[a] += g1
                    for i in 0..<d {
                        g.w0[i][a] += xs[s][i] * g1
                    }
                }
            }
            let c1 = 1 - pow(0.9, Double(t))
            let c2 = 1 - pow(0.999, Double(t))
            adam(&net.w0, &m1.w0, &m2.w0, g.w0, c1, c2)
            adam(&net.b0, &m1.b0, &m2.b0, g.b0, c1, c2)
            adam(&net.w1, &m1.w1, &m2.w1, g.w1, c1, c2)
            adam(&net.b1, &m1.b1, &m2.b1, g.b1, c1, c2)
            adam(&net.w2, &m1.w2, &m2.w2, g.w2, c1, c2)
            adam(&net.b2, &m1.b2, &m2.b2, g.b2, c1, c2)
        }
        return net
    }

    /// One Adam step over a parameter row: weight decay rides on the gradient.
    private static func adam(_ p: inout [Double], _ m1: inout [Double], _ m2: inout [Double],
                             _ g: [Double], _ c1: Double, _ c2: Double) {
        for j in p.indices {
            let gj = g[j] + decay * p[j]
            m1[j] = 0.9 * m1[j] + 0.1 * gj
            m2[j] = 0.999 * m2[j] + 0.001 * gj * gj
            p[j] -= lr * (m1[j] / c1) / (sqrt(m2[j] / c2) + 1e-8)
        }
    }

    /// The same for a weight matrix (each row one parameter row).
    private static func adam(_ p: inout [[Double]], _ m1: inout [[Double]], _ m2: inout [[Double]],
                             _ g: [[Double]], _ c1: Double, _ c2: Double) {
        for i in p.indices {
            adam(&p[i], &m1[i], &m2[i], g[i], c1, c2)
        }
    }
}
