package se.bitcraze.crazyfliecontrol2.gpsnav;

import org.osmdroid.tileprovider.tilesource.OnlineTileSourceBase;
import org.osmdroid.util.MapTileIndex;

/**
 * Tile sources behind the Mission Planner style map backdrop.
 * Nguồn tile cho nền bản đồ theo phong cách Mission Planner.
 *
 * <p>These are Mission Planner's own map providers, ported from GMap.NET's {@code
 * GMap.NET.MapProviders/Google/} classes to osmdroid. Mission Planner draws its Flight Data map
 * through {@code GMapControl}, whose Google layers ask Google for a versioned {@code lyrs=} tile
 * and splice in the subdomain and secure words GMap.NET computes. This class reproduces that
 * request: the same {@code lyrs} versions, the same {@code hl} language, {@code GetServerNum}'s
 * subdomain rotation and {@code GetSecureWords}' "Galileo" tokens.
 * <p>Đây là các nhà cung cấp bản đồ của chính Mission Planner, được chuyển từ các lớp
 * {@code GMap.NET.MapProviders/Google/} của GMap.NET sang osmdroid. Mission Planner vẽ bản đồ
 * Flight Data qua {@code GMapControl}, trong đó các lớp Google của nó hỏi Google lấy tile
 * {@code lyrs=} có gắn phiên bản và ghép thêm subdomain cùng secure word mà GMap.NET tính ra.
 * Lớp này tái tạo đúng request đó: cùng phiên bản {@code lyrs}, cùng ngôn ngữ {@code hl}, phép
 * xoay subdomain của {@code GetServerNum} và các token "Galileo" của {@code GetSecureWords}.
 *
 * <ul>
 *   <li>{@code GoogleSatelliteMapProvider} -> {@link #GOOGLE_SATELLITE}
 *   <li>{@code GoogleHybridMapProvider} -> {@link #GOOGLE_SATELLITE} + {@link #GOOGLE_LABELS}
 *   <li>{@code GoogleMapProvider} -> {@link #GOOGLE_STREET}
 *   <li>{@code GoogleTerrainMapProvider} -> {@link #GOOGLE_TERRAIN}
 * </ul>
 *
 * <p>Hybrid is a composite, exactly as GMap.NET declares it — {@code
 * GoogleHybridMapProvider.Overlays} is {@code [GoogleSatellite, GoogleHybrid]} — so {@link
 * #GOOGLE_LABELS} is drawn over {@link #GOOGLE_SATELLITE} instead of standing alone. See {@link
 * MissionMapView#applyMapType()}.
 * <p>Chế độ hybrid là bản ghép, đúng như GMap.NET khai báo — {@code
 * GoogleHybridMapProvider.Overlays} là {@code [GoogleSatellite, GoogleHybrid]} — nên {@link
 * #GOOGLE_LABELS} được vẽ đè lên {@link #GOOGLE_SATELLITE} chứ không đứng độc lập. Xem {@link
 * MissionMapView#applyMapType()}.
 *
 * <p>One deviation from Mission Planner, and only one: its {@code GoogleSatelliteMapProvider}
 * requests {@code khms{n}.googleapis.com/kh/v=955}, which now answers {@code 404 NOT_FOUND} for
 * every version string tested. Google moved satellite imagery behind the {@code lyrs=s} alias on
 * the same {@code mts} host the other layers use, so the satellite request goes there. The imagery
 * is the same Google satellite layer, on Google's current endpoint.
 * <p>Một điểm lệch so với Mission Planner, và chỉ một: {@code GoogleSatelliteMapProvider} của nó
 * request {@code khms{n}.googleapis.com/kh/v=955}, hiện trả {@code 404 NOT_FOUND} cho mọi chuỗi
 * phiên bản đã thử. Google đã chuyển ảnh vệ tinh sang bí danh {@code lyrs=s} trên cùng host
 * {@code mts} mà các lớp khác dùng, nên request ảnh vệ tinh được gửi sang đó. Lớp ảnh vẫn là
 * lớp vệ tinh của Google, chỉ đổi sang endpoint hiện tại của Google.
 */
final class MapTileSources {

    private MapTileSources() {
    }

    /** GMap.NET's {@code GoogleHybridMapProvider.Version}. */
    /** {@code GoogleHybridMapProvider.Version} của GMap.NET. */
    private static final String HYBRID_VERSION = "h@333000000";

    /** GMap.NET's {@code GoogleMapProvider.Version}. */
    /** {@code GoogleMapProvider.Version} của GMap.NET. */
    private static final String STREET_VERSION = "m@354000000";

    /** GMap.NET's {@code GoogleTerrainMapProvider.Version}. */
    /** {@code GoogleTerrainMapProvider.Version} của GMap.NET. */
    private static final String TERRAIN_VERSION = "t@354,r@354000000";

    /** GMap.NET's {@code LanguageStr} for an English UI. */
    /** {@code LanguageStr} của GMap.NET cho giao diện tiếng Anh. */
    private static final String LANGUAGE = "en";

    /** GMap.NET's {@code GoogleMapProviderBase.SecureWord}, sliced by {@code GetSecureWords}. */
    /** {@code GoogleMapProviderBase.SecureWord} của GMap.NET, được cắt bởi {@code GetSecureWords}. */
    private static final String SECURE_WORD = "Galileo";

    /**
     * GMap.NET's {@code GetServerNum(pos, 4)} — spreads tile requests over {@code mts0}...{@code
     * mts3} so no single host takes the whole load. The subdomains serve identical tiles, so this
     * only affects throughput.
     * {@code GetServerNum(pos, 4)} của GMap.NET — phân tán các request tile ra {@code mts0}...{@code
     * mts3} để không host nào chịu toàn bộ tải. Các subdomain trả về tile giống hệt nhau, nên việc
     * này chỉ ảnh hưởng đến thông lượng.
     */
    private static String serverNum(long mapTileIndex) {
        long x = MapTileIndex.getX(mapTileIndex);
        long y = MapTileIndex.getY(mapTileIndex);
        return String.valueOf((x + 2 * y) % 4);
    }

    /**
     * Builds a tile URL the way GMap.NET's {@code MakeTileImageUrl} does:
     *
     * <pre>{@code
     * https://mts{N}.google.com/maps/vt/lyrs={version}&hl={lang}&x={x}{sec1}&y={y}&z={z}&s={sec2}
     * }</pre>
     *
     * <p>{@code sec1} and {@code sec2} are GMap.NET's {@code GetSecureWords} output: {@code sec2} is
     * a position-dependent prefix of {@code SecureWord}, and {@code sec1} is emitted only for a band
     * of y values. Google ignores both today, but keeping them matches Mission Planner's request
     * byte for byte.
     * <p>{@code sec1} và {@code sec2} là đầu ra của {@code GetSecureWords} trong GMap.NET:
     * {@code sec2} là tiền tố của {@code SecureWord} phụ thuộc vị trí, còn {@code sec1} chỉ được
     * phát cho một dải giá trị y. Ngày nay Google bỏ qua cả hai, nhưng giữ chúng để request khớp
     * với request của Mission Planner từng byte một.
     */
    private static String tileUrl(long mapTileIndex, String version) {
        long x = MapTileIndex.getX(mapTileIndex);
        long y = MapTileIndex.getY(mapTileIndex);
        int zoom = MapTileIndex.getZoom(mapTileIndex);

        // GetSecureWords(pos, out sec1, out sec2)
        // GetSecureWords(pos, out sec1, out sec2) — bản sao của hàm cùng tên trong GMap.NET.
        String sec1 = (y >= 10000 && y < 100000) ? "&s=" : "";
        int secLen = (int) ((x * 3 + y) % 8);
        String sec2 = SECURE_WORD.substring(0, secLen);

        return "https://mts"
                + serverNum(mapTileIndex)
                + ".google.com/maps/vt/lyrs="
                + version
                + "&hl="
                + LANGUAGE
                + "&x="
                + x
                + sec1
                + "&y="
                + y
                + "&z="
                + zoom
                + "&s="
                + sec2;
    }

    private static OnlineTileSourceBase google(String name, String version, String imageEnding) {
        // Google serves this scheme up to z21 across most of the world; past that tiles thin out.
        // Google phục vụ lược đồ này tới z21 trên hầu hết thế giới; vượt quá đó tile thưa dần.
        return new OnlineTileSourceBase(
                name, 0, 21, 256, imageEnding, new String[] {"https://mts0.google.com/maps/vt/"}) {
            @Override
            public String getTileURLString(long mapTileIndex) {
                return tileUrl(mapTileIndex, version);
            }
        };
    }

    /** Google satellite photographs — the base layer of both the satellite and hybrid views. */
    /** Ảnh vệ tinh Google — lớp nền của cả chế độ xem vệ tinh và hybrid. */
    static final OnlineTileSourceBase GOOGLE_SATELLITE = google("GoogleSatellite", "s", ".jpg");

    /** Google place and road labels, drawn over {@link #GOOGLE_SATELLITE} for the hybrid view. */
    /** Nhãn địa danh và đường phố của Google, vẽ đè lên {@link #GOOGLE_SATELLITE} cho chế độ hybrid. */
    static final OnlineTileSourceBase GOOGLE_LABELS =
            google("GoogleHybridLabels", HYBRID_VERSION, ".png");

    /** Google street map. */
    /** Bản đồ đường phố Google. */
    static final OnlineTileSourceBase GOOGLE_STREET = google("GoogleMap", STREET_VERSION, ".png");

    /** Google terrain — shaded relief. */
    /** Địa hình Google — dạng nổi khối đổ bóng. */
    static final OnlineTileSourceBase GOOGLE_TERRAIN =
            google("GoogleTerrain", TERRAIN_VERSION, ".jpg");
}
