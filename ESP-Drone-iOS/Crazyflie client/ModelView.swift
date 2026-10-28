//
//  ModelView.swift
//  Crazyflie client
//
//  The auto-tune model after the latest trial (AutoTuner.TrialReport), a port
//  of PID-Calibration's ModelView.kt. On top every trial's cost (log scale;
//  x = fell, green = best trial so far, dashed = the model's best), below the
//  model's predicted cost over the tuned gains (log axes): a heatmap for two
//  or more gains (others held at the best, dark = better), a curve with a
//  2-std uncertainty band for one. With twiddle the map is only a picture of
//  the trials; it does not steer that search.
//  Dots: blue = this run, grey = earlier runs, red = fell; star = current best.
//

import UIKit

final class ModelView: UIView {

    var report: AutoTuner.TrialReport? {
        didSet {
            heatmapImage = nil
            gridCache = nil
            setNeedsDisplay()
        }
    }

    /// Predicted cost over the tuned gains (AutoTuner.kt's report() grid):
    /// 80 points for one gain (with the 2-std band), 32x32 for two or more.
    private struct Grid {
        let n: Int
        let values: [Double]       // expm1 of the predicted mean
        let lo: [Double]?          // expm1(mean - 2 std), one-gain only
        let hi: [Double]?          // expm1(mean + 2 std), one-gain only
    }

    private var gridCache: Grid?
    private var heatmapImage: CGImage?

    override init(frame: CGRect) {
        super.init(frame: frame)
        backgroundColor = .clear
        isOpaque = false
        contentMode = .redraw
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        backgroundColor = .clear
        isOpaque = false
        contentMode = .redraw
    }

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }
        ctx.setFillColor(UIColor.systemBackground.cgColor)
        ctx.fill(bounds)
        guard let r = report else { return }
        let left: CGFloat = 40
        let right = bounds.width - 10
        let split = bounds.height * 0.36
        drawCosts(r, CGRect(x: left, y: 18, width: right - left, height: split - 36), in: ctx)
        drawModel(r, CGRect(x: left, y: split + 16, width: right - left,
                            height: bounds.height - split - 44), in: ctx)
    }

    // MARK: - Cost panel

    private func drawCosts(_ r: AutoTuner.TrialReport, _ box: CGRect, in ctx: CGContext) {
        let idx = r.own.indices.filter { r.own[$0] }
        let costs = idx.map { r.costs[$0] }
        let fell = idx.map { r.fellMask[$0] }
        let dash = r.method == .twiddle ? "best, mean of 2" : "model's best"
        text("Cost per trial (log; x = fell; dashed = \(dash))", box.minX, box.minY - 6, in: ctx)
        guard !costs.isEmpty else { return }
        let lo = log10(max(0.5, costs.min() ?? 0.5))
        let hi = log10(max(costs.max() ?? 10, 10.0)) + 0.05
        func y(_ v: Double) -> CGFloat {
            box.maxY - CGFloat((log10(max(v, 0.5)) - lo) / (hi - lo)) * box.height
        }
        func x(_ i: Int) -> CGFloat {
            box.minX + (costs.count == 1 ? 0.5 : CGFloat(i) / CGFloat(costs.count - 1)) * box.width
        }
        axes(box, lo, hi, in: ctx)

        ctx.setStrokeColor(UIColor.systemGray4.cgColor)
        ctx.setLineWidth(1)
        ctx.beginPath()
        for (i, v) in costs.enumerated() {
            let p = CGPoint(x: x(i), y: y(v))
            if i == 0 {
                ctx.move(to: p)
            } else {
                ctx.addLine(to: p)
            }
        }
        ctx.strokePath()

        // Best trial so far (falls do not count).
        ctx.setStrokeColor(UIColor.systemGreen.cgColor)
        ctx.beginPath()
        var runMin = Double.infinity
        var prevY: CGFloat?
        for (i, v) in costs.enumerated() {
            if !fell[i] {
                runMin = min(runMin, v)
            }
            guard runMin.isFinite else { continue }
            let yy = y(runMin)
            if let p = prevY {
                ctx.addLine(to: CGPoint(x: x(i), y: p))
                ctx.addLine(to: CGPoint(x: x(i), y: yy))
            } else {
                ctx.move(to: CGPoint(x: x(i), y: yy))
            }
            prevY = yy
        }
        ctx.strokePath()

        if let bp = r.bestPred {
            ctx.setStrokeColor(UIColor.systemGreen.cgColor)
            ctx.setLineDash(phase: 0, lengths: [4, 4])
            ctx.beginPath()
            ctx.move(to: CGPoint(x: box.minX, y: y(bp)))
            ctx.addLine(to: CGPoint(x: box.maxX, y: y(bp)))
            ctx.strokePath()
            ctx.setLineDash(phase: 0, lengths: [])
        }

        for (i, v) in costs.enumerated() {
            dot(CGPoint(x: x(i), y: y(v)), color: fell[i] ? .systemRed : .systemBlue,
                cross: fell[i], in: ctx)
        }
        let label = "trial \(costs.count)"
        text(label, box.maxX - width(of: label), box.maxY + 14, in: ctx)
    }

    /// Log-cost axis: horizontal grid lines at 1, 10, 100 and labels.
    private func axes(_ box: CGRect, _ lo: Double, _ hi: Double, in ctx: CGContext) {
        for e in -1...3 {
            let v = pow(10.0, Double(e))
            let lv = log10(v)
            if lv < lo || lv > hi { continue }
            let yy = box.maxY - CGFloat((lv - lo) / (hi - lo)) * box.height
            ctx.setStrokeColor(UIColor.systemGray4.cgColor)
            ctx.setLineWidth(1)
            ctx.beginPath()
            ctx.move(to: CGPoint(x: box.minX, y: yy))
            ctx.addLine(to: CGPoint(x: box.maxX, y: yy))
            ctx.strokePath()
            let label = String(format: "%.3g", v)
            text(label, box.minX - width(of: label) - 4, yy + 4, in: ctx)
        }
    }

    // MARK: - Model panel

    private func drawModel(_ r: AutoTuner.TrialReport, _ box: CGRect, in ctx: CGContext) {
        if gridCache == nil {
            gridCache = grid(for: r)
        }
        guard let g = gridCache else { return }
        var title: String
        if r.space.names.count == 1 {
            title = "Predicted cost vs \(r.space.names[0]) (band = 2 std)"
        } else {
            title = "Predicted cost, dark = better"
                + (r.space.names.count > 2 ? " (others at best)" : "")
        }
        if r.method == .twiddle {
            title += " - fit for display only"
        } else if r.method != .ml {
            title += " - fit picks the result only"
        }
        text(title, box.minX, box.minY - 6, in: ctx)

        func px(_ u: Double) -> CGFloat { box.minX + CGFloat(u) * box.width }
        func py(_ u: Double) -> CGFloat { box.maxY - CGFloat(u) * box.height }

        let colours = r.xs.indices.map {
            r.fellMask[$0] ? UIColor.systemRed : (r.own[$0] ? .systemBlue : .systemGray2)
        }

        if r.space.names.count == 1 {
            let values = g.values
            let bandLo = g.lo ?? []
            let bandHi = g.hi ?? []
            let lo = log10(max(0.5, min(values.min() ?? 0.5, bandLo.min() ?? 0.5)))
            let hi = log10(max(max(values.max() ?? 10, bandHi.max() ?? 10), 10.0)) + 0.05
            func cy(_ v: Double) -> CGFloat {
                box.maxY - CGFloat((log10(max(v, 0.5)) - lo) / (hi - lo)) * box.height
            }
            func gx(_ i: Int) -> CGFloat { px(Double(i) / Double(g.n - 1)) }
            axes(box, lo, hi, in: ctx)

            // 2-std band.
            ctx.setFillColor(UIColor.systemBlue.withAlphaComponent(0.25).cgColor)
            ctx.beginPath()
            for i in 0..<g.n {
                let p = CGPoint(x: gx(i), y: cy(bandHi[i]))
                if i == 0 {
                    ctx.move(to: p)
                } else {
                    ctx.addLine(to: p)
                }
            }
            for i in stride(from: g.n - 1, through: 0, by: -1) {
                ctx.addLine(to: CGPoint(x: gx(i), y: cy(bandLo[i])))
            }
            ctx.closePath()
            ctx.fillPath()

            ctx.setStrokeColor(UIColor.systemBlue.cgColor)
            ctx.setLineWidth(1.6)
            ctx.beginPath()
            for i in 0..<g.n {
                let p = CGPoint(x: gx(i), y: cy(values[i]))
                if i == 0 {
                    ctx.move(to: p)
                } else {
                    ctx.addLine(to: p)
                }
            }
            ctx.strokePath()

            for (i, x) in r.xs.enumerated() {
                dot(CGPoint(x: px(x[0]), y: cy(r.costs[i])), color: colours[i],
                    cross: r.fellMask[i], in: ctx)
            }
            if let bx = r.bestX {
                star(CGPoint(x: px(bx[0]), y: cy(r.bestPred ?? values.min() ?? 0)), in: ctx)
            }
        } else {
            // Heatmap over the first two gains; any others held at the best.
            if heatmapImage == nil {
                heatmapImage = heatmap(g)
            }
            if let img = heatmapImage {
                ctx.interpolationQuality = .high
                ctx.draw(img, in: box)
            }
            for (i, x) in r.xs.enumerated() {
                dot(CGPoint(x: px(x[0]), y: py(x[1])), color: colours[i],
                    cross: r.fellMask[i], outline: true, in: ctx)
            }
            if let bx = r.bestX {
                star(CGPoint(x: px(bx[0]), y: py(bx[1])), in: ctx)
            }
            // Y axis: the second gain.
            for u in [0.0, 0.5, 1.0] {
                let label = String(format: "%.3g", r.space.value(1, u))
                text(label, box.minX - width(of: label) - 4, py(u) + 4, in: ctx)
            }
            ctx.saveGState()
            ctx.translateBy(x: 10, y: box.midY)
            ctx.rotate(by: -.pi / 2)
            text(r.space.names[1], -width(of: r.space.names[1]) / 2, 0, in: ctx)
            ctx.restoreGState()
        }

        // X axis: the first gain (log scale).
        for u in [0.0, 0.5, 1.0] {
            let label = String(format: "%.3g", r.space.value(0, u))
            let w = width(of: label)
            text(label, min(max(px(u) - w / 2, box.minX), box.maxX - w), box.maxY + 13, in: ctx)
        }
        text(r.space.names[0], box.midX - width(of: r.space.names[0]) / 2, box.maxY + 25, in: ctx)
    }

    /// Predicted cost over the tuned gains, in cost units.
    private func grid(for r: AutoTuner.TrialReport) -> Grid? {
        guard let model = r.model else { return nil }
        let d = r.space.names.count
        let base = r.bestX ?? r.xs.last ?? [Double](repeating: 0.5, count: d)
        if d == 1 {
            let gn = 80
            var lo = [Double](repeating: 0, count: gn)
            var hi = [Double](repeating: 0, count: gn)
            let values = (0..<gn).map { i -> Double in
                let (mean, std) = model.predict([Double(i) / Double(gn - 1)])
                lo[i] = expm1(mean - 2 * std)
                hi[i] = expm1(mean + 2 * std)
                return expm1(mean)
            }
            return Grid(n: gn, values: values, lo: lo, hi: hi)
        }
        let gn = 32
        let values = (0..<(gn * gn)).map { c -> Double in
            var x = base
            x[0] = Double(c % gn) / Double(gn - 1)
            x[1] = Double(c / gn) / Double(gn - 1)
            return expm1(model.predict(x).mean)
        }
        return Grid(n: gn, values: values, lo: nil, hi: nil)
    }

    private func heatmap(_ g: Grid) -> CGImage? {
        let n = g.n
        let logs = g.values.map { log10(max($0, 1.0)) }
        let lo = logs.min() ?? 0
        let hi = max(logs.max() ?? 1, lo + 1e-6)
        var pixels = [UInt8](repeating: 255, count: n * n * 4)
        for i in 0..<(n * n) {
            let gx = i % n
            let gy = n - 1 - i / n                     // row 0 is the top
            let (r8, g8, b8) = ModelView.viridis((logs[gy * n + gx] - lo) / (hi - lo))
            pixels[i * 4] = r8
            pixels[i * 4 + 1] = g8
            pixels[i * 4 + 2] = b8
        }
        let cs = CGColorSpace(name: CGColorSpace.sRGB)!
        guard let ctx = CGContext(data: &pixels, width: n, height: n, bitsPerComponent: 8,
                                  bytesPerRow: n * 4, space: cs,
                                  bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else {
            return nil
        }
        return ctx.makeImage()
    }

    /// Matplotlib's viridis from five stops; 0 = dark purple (low cost).
    private static let viridisStops: [UInt32] =
        [0x440154, 0x3B528B, 0x21918C, 0x5EC962, 0xFDE725]

    private static func viridis(_ t: Double) -> (UInt8, UInt8, UInt8) {
        let f = min(max(t, 0), 1) * Double(viridisStops.count - 1)
        let i = min(Int(f), viridisStops.count - 2)
        let w = f - Double(i)
        func ch(_ shift: UInt32) -> UInt8 {
            let a = Double((viridisStops[i] >> shift) & 0xFF)
            let b = Double((viridisStops[i + 1] >> shift) & 0xFF)
            return UInt8(a * (1 - w) + b * w)
        }
        return (ch(16), ch(8), ch(0))
    }

    // MARK: - Primitives

    private static let textAttrs: [NSAttributedString.Key: Any] = [
        .font: UIFont.systemFont(ofSize: 10),
        .foregroundColor: UIColor.secondaryLabel,
    ]

    private func width(of s: String) -> CGFloat {
        (s as NSString).size(withAttributes: ModelView.textAttrs).width
    }

    /// Kotlin drawText draws with `y` as the baseline; UIKit from the top.
    private func text(_ s: String, _ x: CGFloat, _ y: CGFloat, in ctx: CGContext) {
        UIGraphicsPushContext(ctx)
        (s as NSString).draw(at: CGPoint(x: x, y: y - 8), withAttributes: ModelView.textAttrs)
        UIGraphicsPopContext()
    }

    private func dot(_ p: CGPoint, color: UIColor, cross: Bool, outline: Bool = false,
                     in ctx: CGContext) {
        let d: CGFloat = 3.5
        let rect = CGRect(x: p.x - d, y: p.y - d, width: 2 * d, height: 2 * d)
        if cross {
            ctx.setStrokeColor(color.cgColor)
            ctx.setLineWidth(1.6)
            ctx.beginPath()
            ctx.move(to: CGPoint(x: p.x - d, y: p.y - d))
            ctx.addLine(to: CGPoint(x: p.x + d, y: p.y + d))
            ctx.move(to: CGPoint(x: p.x - d, y: p.y + d))
            ctx.addLine(to: CGPoint(x: p.x + d, y: p.y - d))
            ctx.strokePath()
            return
        }
        ctx.setFillColor(color.cgColor)
        ctx.fillEllipse(in: rect)
        if outline {
            ctx.setStrokeColor(UIColor.white.cgColor)
            ctx.setLineWidth(1)
            ctx.strokeEllipse(in: rect)
        }
    }

    private func star(_ p: CGPoint, in ctx: CGContext) {
        let outer: CGFloat = 8
        let inner = outer * 0.45
        let path = CGMutablePath()
        for k in 0..<10 {
            let a = Double.pi / 2 + Double(k) * Double.pi / 5
            let rad = k % 2 == 0 ? outer : inner
            let q = CGPoint(x: p.x + CGFloat(cos(a)) * rad, y: p.y - CGFloat(sin(a)) * rad)
            if k == 0 {
                path.move(to: q)
            } else {
                path.addLine(to: q)
            }
        }
        path.closeSubpath()
        ctx.setFillColor(UIColor.white.cgColor)
        ctx.addPath(path)
        ctx.fillPath()
        ctx.setStrokeColor(UIColor.black.cgColor)
        ctx.setLineWidth(1)
        ctx.addPath(path)
        ctx.strokePath()
    }
}
