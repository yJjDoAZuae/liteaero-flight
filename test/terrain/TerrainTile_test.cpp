#define _USE_MATH_DEFINES
#include <liteaero/terrain/TerrainTile.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

using namespace liteaero::terrain;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Minimal valid tile: an equilateral-ish right triangle at equator/prime-meridian,
// all vertices at up_m = 0.
static TerrainTile makeMinimalTile(TerrainLod lod) {
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-0.001, 0.001, -0.001, 0.001, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{
        { 0.f,  0.f,  0.f},
        {10.f,  0.f,  0.f},
        { 0.f, 10.f,  0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {128, 64, 32}}};
    return TerrainTile(lod, centroid, bounds, std::move(vertices), std::move(facets));
}

// ---------------------------------------------------------------------------
// T1: Constructor accessors
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, ConstructorAccessors) {
    GeodeticPoint centroid{0.1, 0.2, 50.f};
    GeodeticAABB  bounds{0.09, 0.11, 0.19, 0.21, 45.f, 55.f};
    std::vector<TerrainVertex> vertices{
        {0.f, 0.f,  0.f},
        {5.f, 0.f,  0.f},
        {0.f, 5.f,  0.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {10, 20, 30}}};

    TerrainTile tile(TerrainLod::L2_HighDetail, centroid, bounds,
                     std::move(vertices), std::move(facets));

    EXPECT_EQ(tile.lod(), TerrainLod::L2_HighDetail);
    EXPECT_DOUBLE_EQ(tile.centroid().latitude_rad,  0.1);
    EXPECT_DOUBLE_EQ(tile.centroid().longitude_rad, 0.2);
    EXPECT_FLOAT_EQ(tile.centroid().height_wgs84_m, 50.f);
    EXPECT_EQ(tile.vertices().size(), 3u);
    EXPECT_EQ(tile.facets().size(),   1u);
    EXPECT_DOUBLE_EQ(tile.bounds().lat_min_rad, 0.09);
    EXPECT_DOUBLE_EQ(tile.bounds().lat_max_rad, 0.11);
}

// ---------------------------------------------------------------------------
// T2: facetCentroid for a right-triangle tile at the equator/prime-meridian
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, FacetCentroid_RightTriangle) {
    // Centroid at equator, prime meridian, h=0.
    // Vertices form a right triangle with ENU offsets (all at up=100 m):
    //   V0=(0,0,100), V1=(100,0,100), V2=(0,100,100)
    // Average ENU: (33.333, 33.333, 100).
    // Expected geodetic: lat ≈ 33.333/R_earth, lon ≈ 33.333/R_earth, h ≈ 100 m
    GeodeticPoint centroid{0.0, 0.0, 0.f};
    GeodeticAABB  bounds{-0.0001, 0.0001, -0.0001, 0.0001, 90.f, 110.f};
    std::vector<TerrainVertex> vertices{
        {  0.f,   0.f, 100.f},
        {100.f,   0.f, 100.f},
        {  0.f, 100.f, 100.f},
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    const GeodeticPoint c = tile.facetCentroid(0);

    // At the equator, north offsets use the meridional radius M = a*(1-e²),
    // while east offsets use the prime vertical radius N = a (since cos(0) = 1).
    const double kA  = 6378137.0;
    const double kE2 = 6.69437999014e-3;
    const double expected_lat = (100.0 / 3.0) / (kA * (1.0 - kE2));  // ≈ 5.26e-6 rad
    const double expected_lon = (100.0 / 3.0) / kA;                   // ≈ 5.23e-6 rad
    EXPECT_NEAR(c.latitude_rad,  expected_lat, 1e-9);
    EXPECT_NEAR(c.longitude_rad, expected_lon, 1e-9);
    EXPECT_NEAR(c.height_wgs84_m, 100.f, 0.01f);
}

// ---------------------------------------------------------------------------
// T3: facetNormal for a horizontal face near the north pole has ECEF Z > 0.99
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, FacetNormal_HorizontalFace) {
    // Tile centroid at 89°N, 0°E so that ECEF Z ≈ sin(89°) ≈ 0.9998 > 0.99.
    // CCW winding when viewed from above: V0→V1 (east), V1→V2 (northwest),
    // cross product points up (+ENU Up → +ECEF Z at high latitude).
    const double lat = 89.0 * M_PI / 180.0;
    GeodeticPoint centroid{lat, 0.0, 0.f};
    GeodeticAABB  bounds{lat - 0.001, lat + 0.001, -0.001, 0.001, -1.f, 1.f};
    std::vector<TerrainVertex> vertices{
        {  0.f,   0.f, 0.f},   // V0
        {100.f,   0.f, 0.f},   // V1 (100 m east of V0)
        {  0.f, 100.f, 0.f},   // V2 (100 m north of V0)
    };
    std::vector<TerrainFacet> facets{{{0, 1, 2}, {0, 0, 0}}};
    TerrainTile tile(TerrainLod::L0_Finest, centroid, bounds,
                     std::move(vertices), std::move(facets));

    const auto normal = tile.facetNormal(0);
    EXPECT_GT(normal[2], 0.99);  // ECEF Z ≈ sin(89°) ≈ 0.9998
    // Verify it is a unit vector.
    const double len = std::sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
    EXPECT_NEAR(len, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// T4: JSON round-trip preserves all vertex ENU values and facet indices
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, JsonRoundTrip_VerticesAndIndicesPreserved) {
    GeodeticPoint centroid{0.3, 0.5, 200.f};
    GeodeticAABB  bounds{0.29, 0.31, 0.49, 0.51, 190.f, 210.f};
    std::vector<TerrainVertex> vertices{
        {  1.5f,   2.5f,  3.5f},
        {-10.0f,   5.0f, 20.0f},
        { 30.0f, -15.0f,  0.0f},
    };
    std::vector<TerrainFacet> facets{{{0, 2, 1}, {255, 128, 0}}};
    TerrainTile original(TerrainLod::L3_MediumDetail, centroid, bounds,
                         std::move(vertices), std::move(facets));

    const nlohmann::json j = original.serializeJson();
    const TerrainTile restored = TerrainTile::deserializeJson(j);

    EXPECT_EQ(restored.lod(), TerrainLod::L3_MediumDetail);
    ASSERT_EQ(restored.vertices().size(), 3u);
    EXPECT_NEAR(restored.vertices()[0].east_m,   1.5f,  1e-4f);
    EXPECT_NEAR(restored.vertices()[0].north_m,  2.5f,  1e-4f);
    EXPECT_NEAR(restored.vertices()[0].up_m,     3.5f,  1e-4f);
    EXPECT_NEAR(restored.vertices()[1].east_m, -10.0f,  1e-4f);
    EXPECT_NEAR(restored.vertices()[2].north_m,-15.0f,  1e-4f);
    ASSERT_EQ(restored.facets().size(), 1u);
    EXPECT_EQ(restored.facets()[0].v[0], 0u);
    EXPECT_EQ(restored.facets()[0].v[1], 2u);
    EXPECT_EQ(restored.facets()[0].v[2], 1u);
    EXPECT_EQ(restored.facets()[0].color.r, 255);
    EXPECT_EQ(restored.facets()[0].color.g, 128);
    EXPECT_EQ(restored.facets()[0].color.b, 0);
    EXPECT_DOUBLE_EQ(restored.centroid().latitude_rad,  0.3);
    EXPECT_DOUBLE_EQ(restored.centroid().longitude_rad, 0.5);
    EXPECT_FLOAT_EQ(restored.centroid().height_wgs84_m, 200.f);
}

// ---------------------------------------------------------------------------
// T5: JSON schema version mismatch throws std::runtime_error
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, JsonSchemaVersionMismatch_Throws) {
    const TerrainTile tile = makeMinimalTile(TerrainLod::L1_VeryHighDetail);
    nlohmann::json j = tile.serializeJson();
    j["schema_version"] = 99;
    EXPECT_THROW({ (void)TerrainTile::deserializeJson(j); }, std::runtime_error);
}

// ---------------------------------------------------------------------------
// T6: Proto round-trip preserves vertices, facets, centroid, and LOD
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, ProtoRoundTrip_VerticesAndIndicesPreserved) {
    GeodeticPoint centroid{0.6, 0.8, 150.f};
    GeodeticAABB  bounds{0.59, 0.61, 0.79, 0.81, 140.f, 160.f};
    std::vector<TerrainVertex> vertices{
        { 5.0f, -3.0f, 10.0f},
        {-2.0f,  7.0f,  0.0f},
        {12.0f,  4.0f,  5.0f},
    };
    std::vector<TerrainFacet> facets{{{2, 0, 1}, {100, 150, 200}}};
    TerrainTile original(TerrainLod::L4_LowDetail, centroid, bounds,
                         std::move(vertices), std::move(facets));

    const std::vector<uint8_t> bytes = original.serializeProto();
    const TerrainTile restored = TerrainTile::deserializeProto(bytes);

    EXPECT_EQ(restored.lod(), TerrainLod::L4_LowDetail);
    ASSERT_EQ(restored.vertices().size(), 3u);
    EXPECT_NEAR(restored.vertices()[0].east_m,   5.0f, 1e-4f);
    EXPECT_NEAR(restored.vertices()[0].north_m, -3.0f, 1e-4f);
    EXPECT_NEAR(restored.vertices()[0].up_m,    10.0f, 1e-4f);
    EXPECT_NEAR(restored.vertices()[1].east_m,  -2.0f, 1e-4f);
    ASSERT_EQ(restored.facets().size(), 1u);
    EXPECT_EQ(restored.facets()[0].v[0], 2u);
    EXPECT_EQ(restored.facets()[0].v[1], 0u);
    EXPECT_EQ(restored.facets()[0].v[2], 1u);
    EXPECT_EQ(restored.facets()[0].color.r, 100);
    EXPECT_EQ(restored.facets()[0].color.g, 150);
    EXPECT_EQ(restored.facets()[0].color.b, 200);
    EXPECT_DOUBLE_EQ(restored.centroid().latitude_rad,  0.6);
    EXPECT_DOUBLE_EQ(restored.centroid().longitude_rad, 0.8);
    EXPECT_FLOAT_EQ(restored.centroid().height_wgs84_m, 150.f);
}

// ---------------------------------------------------------------------------
// T7: Proto schema version mismatch throws std::runtime_error
// ---------------------------------------------------------------------------
TEST(TerrainTileTest, ProtoSchemaVersionMismatch_Throws) {
    // Serialize a valid tile, then corrupt the bytes by re-serializing with
    // schema_version = 99 via JSON → proto path is not available, so we test
    // by directly passing a tile that produces version=1 and verify the happy
    // path round-trips; the mismatch path is tested via a hand-crafted proto.
    // Build a minimal proto with schema_version=99 manually.
    const TerrainTile tile = makeMinimalTile(TerrainLod::L0_Finest);
    const std::vector<uint8_t> good_bytes = tile.serializeProto();
    // Corrupt: replace schema_version field.  Proto field 1 (int32) is varint
    // tag 0x08 followed by the value.  The first two bytes of a well-formed
    // TerrainTileProto with schema_version=1 are 0x08 0x01.  Replace with 99.
    std::vector<uint8_t> bad_bytes = good_bytes;
    // Find and patch the schema_version varint (tag=0x08, value=0x01 → 0x63).
    for (std::size_t i = 0; i + 1 < bad_bytes.size(); ++i) {
        if (bad_bytes[i] == 0x08 && bad_bytes[i + 1] == 0x01) {
            bad_bytes[i + 1] = 0x63;  // 99 decimal
            break;
        }
    }
    EXPECT_THROW({ (void)TerrainTile::deserializeProto(bad_bytes); }, std::runtime_error);
}
