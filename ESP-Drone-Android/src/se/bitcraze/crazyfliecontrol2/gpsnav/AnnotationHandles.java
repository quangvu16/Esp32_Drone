package se.bitcraze.crazyfliecontrol2.gpsnav;

import android.graphics.Canvas;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Shader;

import androidx.core.graphics.PathParser;

/**
 * The on-map control handles of Valetudo's map-annotation editor: the red ✕ that deletes, the grey
 * ✛ that moves a line's endpoint, the ↻ that rotates a quad and the ↖↘ that scales it. Ported
 * 1:1 from {@code hypfer/valetudo} {@code frontend/src/map/structures/icons/*_zone.svg} — the same
 * 32×32 viewBox, the same circle gradients and the same glyph path data.
 *
 * <p>Các tay nắm điều khiển trên bản đồ của trình sửa chú thích Valetudo: ✕ đỏ để xoá, ✛ xám để
 * kéo đầu mút của đường, ↻ để xoay khối và ↖↘ để co giãn. Chuyển thể 1:1 từ
 * {@code hypfer/valetudo} {@code frontend/src/map/structures/icons/*_zone.svg} — cùng khung nhìn
 * 32×32, cùng gradient của vòng tròn và cùng dữ liệu đường vẽ của biểu tượng.
 *
 * <p>Valetudo rotates the whole icon with the shape's angle when drawing
 * ({@code drawRotatedIcon}); {@link #draw} does the same via the {@code angleDegrees} argument.
 *
 * <p>Valetudo xoay cả biểu tượng theo góc của hình khi vẽ ({@code drawRotatedIcon});
 * {@link #draw} làm vậy qua tham số {@code angleDegrees}.
 */
final class AnnotationHandles {

    /** The four control icons (bốn biểu tượng điều khiển). */
    static final int DELETE = 0;
    static final int MOVE = 1;
    static final int ROTATE = 2;
    static final int SCALE = 3;

    /*
     * Glyph path data copied verbatim from the SVGs' <path d="..."> (32×32 user space).
     * Dữ liệu đường vẽ chép nguyên từ <path d="..."> của các SVG (không gian 32×32).
     */
    private static final String GLYPH_DELETE =
            "m9.4609 7.4492-2.0117 2.0117 6.5508 6.5508-6.5508 6.5488 2.0117 2.0117 6.5508-6.5488"
            + " 6.5488 6.5488 2.0117-2.0117-6.5488-6.5488 6.5488-6.5508-2.0117-2.0117-6.5488"
            + " 6.5508-6.5508-6.5508z";

    private static final String GLYPH_MOVE =
            "m16.01 2.8652-1.3555 2.3477-0.066406 0.11523-1.2891 2.2324h1.2891v7.0313h-7.0293"
            + "v-1.2871l-2.2324 1.2871h-0.0019531v0.001953l-0.11328 0.064453-2.3477 1.3555"
            + " 2.4609 1.4199v0.003906h0.0058593l2.2285 1.2871v-1.2871h7.0293v7.0215h-1.2871"
            + "l1.2871 2.2305v0.009766h0.005859l1.416 2.4531 1.3555-2.3457 0.0625-0.10742h0.003907"
            + "v-0.007813l1.2891-2.2324h-1.2891v-7.0215h7.0371v1.2871l4.6953-2.7109-2.3477-1.3555"
            + "-2.3477-1.3535v1.2871h-7.0371v-7.0313h1.2871l-1.2871-2.2305v-0.0019531l-1.4219-2.4629z";

    private static final String GLYPH_ROTATE =
            "m9.3027 7.1855-2.4102 1.4258h-0.0019531v0.0019531l-0.12305 0.070313-2.5352 1.5"
            + " 2.6582 1.5723v0.003907h0.0058594l2.4062 1.4238v-1.4238-0.18164a1.4 1.4 0 0 0"
            + " 0.09375 0.003906h2.6523c4.5179 0 8.1445 3.6964 8.1445 8.3828v2.7188a1.4 1.4 0 0 0"
            + " 0 0.035156h-0.13867-1.3906l1.3906 2.4688v0.009766h0.005859l1.5293 2.7148"
            + " 1.4629-2.5957 0.06836-0.11914h0.003906v-0.007813l1.3926-2.4707h-1.3926-0.13086a1.4"
            + " 1.4 0 0 0 0-0.035156v-2.7188c0-6.1527-4.8873-11.184-10.945-11.184h-2.6523a1.4 1.4"
            + " 0 0 0-0.09375 0.0039062v-0.17383-1.4258z";

    private static final String GLYPH_SCALE =
            "m6.6523 6.6523 0.70536 2.6328 0.036447 0.13507-0.00214 0.0022 0.00429 0.0043"
            + " 0.66677 2.4934 0.91333-0.91333 12.028 12.03-0.91118 0.91118 2.4956 0.67106"
            + " 0.0022 0.0022 0.0022-0.0022 0.13293 0.03644 2.6328 0.70536-0.74181-2.7657"
            + " 0.0043-0.0043-0.0065-0.0065-0.66677-2.4913-0.91118 0.91118-12.03-12.028"
            + " 0.91118-0.91333-2.4913-0.66677-0.0043-0.00429-0.0022 0.00214z";

    /** The parsed glyphs in 32×32 space (các biểu tượng đã phân tích, không gian 32×32). */
    private static final Path[] GLYPHS = new Path[4];

    static {
        GLYPHS[DELETE] = PathParser.createPathFromPathData(GLYPH_DELETE);
        GLYPHS[MOVE] = PathParser.createPathFromPathData(GLYPH_MOVE);
        GLYPHS[ROTATE] = PathParser.createPathFromPathData(GLYPH_ROTATE);
        GLYPHS[SCALE] = PathParser.createPathFromPathData(GLYPH_SCALE);
    }

    private static final float ICON_BOX = 32f;
    private static final float CIRCLE_CX = 16f;
    private static final float CIRCLE_CY = 16f;
    private static final float CIRCLE_RADIUS = 14.946f;
    private static final float CIRCLE_STROKE = 2.1087f;

    private AnnotationHandles() {
    }

    /**
     * Draw one handle centred on (cx, cy), {@code size} pixels across, rotated by
     * {@code angleDegrees} exactly like Valetudo's {@code drawRotatedIcon}.
     *
     * <p>Vẽ một tay nắm tại tâm (cx, cy), bề ngang {@code size} pixel, xoay {@code angleDegrees}
     * đúng như {@code drawRotatedIcon} của Valetudo.
     */
    static void draw(Canvas canvas, int kind, float cx, float cy, float size, float angleDegrees) {
        float scale = size / ICON_BOX;
        canvas.save();
        canvas.translate(cx, cy);
        canvas.rotate(angleDegrees);
        canvas.scale(scale, scale);
        canvas.translate(-CIRCLE_CX, -CIRCLE_CY);

        Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
        fill.setStyle(Paint.Style.FILL);
        // Circle gradients straight from the SVGs' <linearGradient> defs.
        // Gradient của vòng tròn lấy thẳng từ phần <linearGradient> của các SVG.
        if (kind == DELETE) {
            fill.setShader(new LinearGradient(27.32f, 4.692f, 4.692f, 27.32f,
                    0xFFF15B5B, 0xFFEF4444, Shader.TileMode.CLAMP));
        } else {
            fill.setShader(new LinearGradient(27.32f, 4.692f, 4.692f, 27.32f,
                    0xFF8C8C8C, 0xFF808080, Shader.TileMode.CLAMP));
        }

        Paint border = new Paint(Paint.ANTI_ALIAS_FLAG);
        border.setStyle(Paint.Style.STROKE);
        border.setStrokeWidth(CIRCLE_STROKE);
        border.setShader(new LinearGradient(27.32f, 4.692f, 4.692f, 27.32f,
                0xFF404040, 0xFF333333, Shader.TileMode.CLAMP));

        Paint glyphPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        glyphPaint.setStyle(Paint.Style.FILL);
        glyphPaint.setShader(new LinearGradient(22.065f, 9.9568f, 9.9568f, 22.065f,
                0xFF404040, 0xFF333333, Shader.TileMode.CLAMP));

        canvas.drawCircle(CIRCLE_CX, CIRCLE_CY, CIRCLE_RADIUS, fill);
        canvas.drawCircle(CIRCLE_CX, CIRCLE_CY, CIRCLE_RADIUS, border);
        if (kind >= 0 && kind < GLYPHS.length && GLYPHS[kind] != null) {
            canvas.drawPath(GLYPHS[kind], glyphPaint);
        }
        canvas.restore();
    }
}
