//
//  ChartView.swift
//  Crazyflie client
//
//  Scrolling strip chart, ported from PID-Calibration's ChartView.kt.
//  Every series is scaled on its own, symmetric around zero, so a small angle
//  and a ±255 controller output are both readable; the legend gives each
//  series' range. With `sharedScale` all series use one range instead, so they
//  compare directly (the per-term panels). The zero line is the middle of the
//  plot.
//
//  Capacity is 300 samples at 20 Hz = 15 s, matching the Android client.
//

import UIKit

final class ChartView: UIView {

    struct Series {
        let name: String
        let unit: String
        let color: UIColor
        /// Floor on the plotted range so a flat line is still readable.
        let minRange: Float
        var enabled: Bool
    }

    static let capacity = 300

    var series: [Series] = [] {
        didSet {
            data = Array(repeating: [Float](repeating: 0, count: ChartView.capacity), count: series.count)
            head = 0
            count = 0
            setNeedsDisplay()
        }
    }

    var paused = false {
        didSet { setNeedsDisplay() }
    }

    var title: String?
    var sharedScale = false
    /// Shown once in the legend when `sharedScale`.
    var unit = ""
    /// Dashed horizontal lines, e.g. PWM saturation limits.
    var markers: [Float] = []

    private var data: [[Float]] = []
    /// Index the next sample goes to.
    private var head = 0
    private var count = 0

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

    override var intrinsicContentSize: CGSize {
        CGSize(width: UIView.noIntrinsicMetric, height: 220)
    }

    func add(_ values: [Float]) {
        guard !paused, !data.isEmpty else { return }
        for i in 0..<min(values.count, data.count) {
            data[i][head] = values[i]
        }
        head = (head + 1) % ChartView.capacity
        if count < ChartView.capacity { count += 1 }
        setNeedsDisplay()
    }

    func clear() {
        count = 0
        head = 0
        setNeedsDisplay()
    }

    /// k-th oldest stored sample of series i.
    private func at(_ i: Int, _ k: Int) -> Float {
        data[i][(head - count + k + ChartView.capacity) % ChartView.capacity]
    }

    private func fmt(_ v: Float) -> String {
        if abs(v) >= 100 { return String(format: "%.0f", v) }
        if abs(v) >= 10 { return String(format: "%.1f", v) }
        return String(format: "%.2f", v)
    }

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }
        let w = bounds.width
        let h = bounds.height
        let bg = UIColor.secondarySystemBackground
        ctx.setFillColor(bg.cgColor)
        ctx.fill(bounds)

        let top: CGFloat = sharedScale ? 22 : 38
        let bottom = h - 6
        let mid = (top + bottom) / 2
        let half = (bottom - top) / 2

        let gridColor = UIColor.separator.cgColor
        for k in -2...2 {
            let y = mid + CGFloat(k) * half / 2
            ctx.setStrokeColor(gridColor)
            ctx.setAlpha(k == 0 ? 1.0 : 0.35)
            ctx.setLineWidth(1)
            ctx.move(to: CGPoint(x: 0, y: y))
            ctx.addLine(to: CGPoint(x: w, y: y))
            ctx.strokePath()
            ctx.setAlpha(1)
        }
        // 1 s ticks at 20 Hz
        ctx.setStrokeColor(gridColor)
        ctx.setAlpha(0.25)
        var s = 0
        while s < ChartView.capacity {
            let x = w - CGFloat(s) * w / CGFloat(ChartView.capacity - 1)
            ctx.move(to: CGPoint(x: x, y: top))
            ctx.addLine(to: CGPoint(x: x, y: bottom))
            ctx.strokePath()
            s += 20
        }
        ctx.setAlpha(1)

        let font = UIFont.monospacedSystemFont(ofSize: 11, weight: .regular)
        let mutedAttrs: [NSAttributedString.Key: Any] = [
            .font: font,
            .foregroundColor: UIColor.secondaryLabel,
        ]
        var lx: CGFloat = 8
        var ly: CGFloat = 6

        var shared: Float = 0
        if sharedScale {
            for i in 0..<series.count where series[i].enabled {
                shared = max(shared, series[i].minRange)
                for k in 0..<count {
                    shared = max(shared, abs(at(i, k)))
                }
            }
            for m in markers where abs(m) <= shared {
                let y = mid - CGFloat(m / shared) * half
                ctx.setStrokeColor(UIColor.systemRed.cgColor)
                ctx.setAlpha(0.7)
                ctx.setLineWidth(1)
                ctx.setLineDash(phase: 0, lengths: [4, 4])
                ctx.move(to: CGPoint(x: 0, y: y))
                ctx.addLine(to: CGPoint(x: w, y: y))
                ctx.strokePath()
                ctx.setLineDash(phase: 0, lengths: [])
                ctx.setAlpha(1)
            }
            let head = ((title != nil ? "\(title!) " : "") + "±" + fmt(shared) + unit)
            (head as NSString).draw(at: CGPoint(x: lx, y: ly), withAttributes: mutedAttrs)
            lx += (head as NSString).size(withAttributes: mutedAttrs).width + 12
        }

        for i in 0..<series.count {
            let ser = series[i]
            guard ser.enabled else { continue }
            let range: Float = sharedScale ? shared : {
                var r = ser.minRange
                for k in 0..<count { r = max(r, abs(at(i, k))) }
                return r
            }()

            if count > 1 {
                let path = CGMutablePath()
                for k in 0..<count {
                    // newest sample at the right edge
                    let x = w - CGFloat(count - 1 - k) * w / CGFloat(ChartView.capacity - 1)
                    let y = mid - CGFloat(at(i, k) / range) * half
                    if k == 0 {
                        path.move(to: CGPoint(x: x, y: y))
                    } else {
                        path.addLine(to: CGPoint(x: x, y: y))
                    }
                }
                ctx.addPath(path)
                ctx.setStrokeColor(ser.color.cgColor)
                ctx.setLineWidth(2)
                ctx.setLineJoin(.round)
                ctx.strokePath()
            }

            let label = sharedScale ? ser.name : "\(ser.name) ±\(fmt(range))\(ser.unit)"
            let attrs: [NSAttributedString.Key: Any] = [
                .font: font,
                .foregroundColor: ser.color,
            ]
            let tw = (label as NSString).size(withAttributes: attrs).width + 12
            if lx + tw > w {
                lx = 8
                ly += 14
            }
            (label as NSString).draw(at: CGPoint(x: lx, y: ly), withAttributes: attrs)
            lx += tw
        }

        if count == 0 {
            let msg = "Waiting for telemetry" as NSString
            let sz = msg.size(withAttributes: mutedAttrs)
            msg.draw(at: CGPoint(x: (w - sz.width) / 2, y: mid - 6), withAttributes: mutedAttrs)
        }
        if paused {
            let msg = "PAUSED" as NSString
            let sz = msg.size(withAttributes: mutedAttrs)
            msg.draw(at: CGPoint(x: w - sz.width - 8, y: bottom - 16), withAttributes: mutedAttrs)
        }
    }
}

// MARK: - Palette
//
// Fixed categorical order from the dataviz reference palette. These four steps
// are the documented dark-surface set (the validator could not run here — no
// node — but the reference files these as passing for these exact slots).
// Color follows the entity (series identity), never its rank in the list.

extension ChartView {
    static let seriesColors: [UIColor] = [
        UIColor(red: 0x39/255, green: 0x87/255, blue: 0xe5/255, alpha: 1), // #3987e5
        UIColor(red: 0xd9/255, green: 0x59/255, blue: 0x26/255, alpha: 1), // #d95926
        UIColor(red: 0x19/255, green: 0x9e/255, blue: 0x70/255, alpha: 1), // #199e70
        UIColor(red: 0xc9/255, green: 0x85/255, blue: 0x00/255, alpha: 1), // #c98500
    ]

    static func color(at index: Int) -> UIColor {
        seriesColors[index % seriesColors.count]
    }
}
