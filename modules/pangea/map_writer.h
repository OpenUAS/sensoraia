/*
 * Copyright 2026 OpenUAS
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file map_writer.h
 * @brief Write georeferencing sidecars for rendered Pangea map images.
 * @details The world file and PAM metadata use different origin conventions:
 * the world file identifies the centre of the upper-left pixel, while the
 * GeoTransform identifies its outer corner. Keeping both values derived from
 * the same map grid prevents GIS readers from shifting the image by half a
 * pixel. Sidecars are published without overwriting prior work so a failed run
 * cannot silently replace metadata belonging to another map.
 */
#ifndef PANGEA_MAP_WRITER_H
#define PANGEA_MAP_WRITER_H

#include "map_projection.h"
#include "application_runtime.h"

namespace pangea_survey {
/**
 * @brief Create the georeferencing files paired with a rendered map image.
 * @details This writer owns only the .jgw and .aux.xml sidecars; JPEG encoding
 * and publication are handled separately. The sidecars describe the requested
 * grid, not surveyed ground truth, and are not a claim of orthorectified relief.
 */
class MapWriter {
    /** @brief Write and publish one complete georeferencing sidecar.
     * @param path Final sidecar path in the caller's working directory.
     * @param text Serialized world-file or XML content.
     * @throws std::runtime_error On an existing path or write/publication failure.
     * @details Stage under a sibling name before renaming. The caller
     * coordinates exclusive access and any cleanup of the sidecar pair. */
    static void writeMetadata(const fs::path& path, const std::string& text) {
        const fs::path temporary = path.string() + ".partial";
        if (fs::exists(path) || fs::exists(temporary))
            throw std::runtime_error("Refusing to overwrite map metadata: " + path.u8string());
        try {
            // Rename exposes only a complete sidecar; the caller still owns pair-level cleanup.
            std::ofstream output(temporary, std::ios::binary);
            output << text;
            output.close();
            if (!output) throw std::runtime_error("Cannot write map metadata: " + path.u8string());
            std::error_code ec;
            fs::rename(temporary, path, ec);
            if (ec) throw std::runtime_error("Finalize map metadata failed: " + ec.message());
        } catch (...) {
            std::error_code error;
            fs::remove(temporary, error);
            if (error) std::cerr << "WARNING: Cannot remove partial map metadata: " << error.message() << '\n';
            throw;
        }
    }
public:
    /**
     * @brief Return the two sidecar paths associated with a map image.
     * @param[in] path Image path whose extension is replaced for the world file.
     * @return World-file path followed by the PAM XML path.
     */
    static std::array<fs::path, 2> mapFiles(const fs::path& path) {
        auto world = path;
        world.replace_extension(".jgw");
        return {world, fs::path(path.string() + ".aux.xml")};
    }

    /**
     * @brief Publish a world file and GIS projection metadata for one map image.
     * @param[in] path Destination image path used to derive both sidecar names.
     * @param[in] map Projected map centre, pixel size, heading, and EPSG code.
     * @param[in] width Image width in pixels; used to place the grid origin.
     * @param[in] height Image height in pixels; used to place the grid origin.
     * @param[in] orthorectified Whether the PAM metadata may describe surface
     * correction; false deliberately labels the output as reference-plane only.
     * @throws std::runtime_error If either sidecar already exists or cannot be
     * written. If the second file fails, the first is removed to avoid leaving
     * a misleading half-pair.
     *
     * The world-file origin is the upper-left pixel centre. The PAM
     * GeoTransform starts at the upper-left pixel corner, as required by GDAL;
     * these values differ by half a pixel by design.
     */
    static void writeMap(const fs::path& path, const pangea_survey::MapTile& map, int width, int height,
                         bool orthorectified = false) {
        const auto paths = mapFiles(path);
        std::ostringstream world, xml;
        world.imbue(std::locale::classic());
        xml.imbue(std::locale::classic());
        world << std::setprecision(15) << map.pixelSize << "\n0\n0\n" << -map.pixelSize << '\n'
              << map.east - (width - 1) * map.pixelSize / 2 << '\n'
              << map.north + (height - 1) * map.pixelSize / 2 << '\n';
        const int zone = map.epsg % 100;
        xml << std::setprecision(15)
            << "<PAMDataset><SRS>PROJCS[&quot;WGS 84 / UTM zone " << zone
            << (map.epsg < 32700 ? "N" : "S")
            << "&quot;,GEOGCS[&quot;WGS 84&quot;,DATUM[&quot;WGS_1984&quot;,SPHEROID[&quot;WGS 84&quot;,6378137,298.257223563]],"
               "PRIMEM[&quot;Greenwich&quot;,0],UNIT[&quot;degree&quot;,0.0174532925199433]],"
               "PROJECTION[&quot;Transverse_Mercator&quot;],PARAMETER[&quot;latitude_of_origin&quot;,0],"
               "PARAMETER[&quot;central_meridian&quot;," << zone * 6 - 183
            << "],PARAMETER[&quot;scale_factor&quot;,0.9996],PARAMETER[&quot;false_easting&quot;,500000],"
               "PARAMETER[&quot;false_northing&quot;," << (map.epsg < 32700 ? 0 : 10000000)
            << "],UNIT[&quot;metre&quot;,1],AUTHORITY[&quot;EPSG&quot;,&quot;" << map.epsg
            << "&quot;]]</SRS><GeoTransform>"
            << map.east - width * map.pixelSize / 2 << ',' << map.pixelSize << ",0,"
            << map.north + height * map.pixelSize / 2 << ",0," << -map.pixelSize
            << "</GeoTransform><Metadata><MDI key=\"GEOREFERENCING_MODEL\">"
            << (orthorectified ? "Multi-view surface orthorectification; see geometry metadata for coverage classes" :
                                 "Reference-plane georeferencing; not a true orthophoto")
            << "</MDI></Metadata></PAMDataset>\n";
        writeMetadata(paths[0], world.str());
        try { writeMetadata(paths[1], xml.str()); }
        catch (...) {
            std::error_code error;
            fs::remove(paths[0], error);
            if (error) std::cerr << "WARNING: Cannot remove incomplete map metadata: " << error.message() << '\n';
            throw;
        }
    }
};
}
#endif
