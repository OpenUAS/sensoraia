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
 * @file map_exif.h
 * @brief Add projection, coverage, and source provenance to map EXIF.
 * @details Large source lists move to a companion JSON file rather than
 * overflowing the bounded EXIF payload. Metadata also labels the output's
 * reference-plane model and simulator-coordinate accuracy limits explicitly.
 */
#ifndef PANGEA_MAP_EXIF_H
#define PANGEA_MAP_EXIF_H

#include "photo_exif.h"
#include "ortho_map.h"
namespace pangea_survey {
/**
 * @brief Serialize source-camera provenance and map-grid metadata for orthophotos.
 * @details This is derived-map metadata: it does not claim exposure-time pose
 * synchronization, a measured lens calibration, or relief-corrected geometry.
 */
class MapExif : public PhotoExif {
public:
    /**
     * @brief Encode each source image's optical, geographic, and timing provenance.
     * @param[in] sources Sources contributing to one rendered map.
     * @return JSON array suitable for an EXIF comment or companion file.
     */
    static std::string sourceMetadata(const std::vector<OrthoSource>& sources) {
        std::ostringstream json;
        json.imbue(std::locale::classic()); json << std::setprecision(15) << '[';
        bool first = true;
        for (const auto& source : sources) {
            if (!first) json << ',';
            first = false;
            json << "{\"filename\":" << jsonString(source.path.filename().u8string())
                 << ",\"lens_model\":\"SimConnect rectilinear virtual lens\","
                    "\"projection\":\"perspective\",\"sdk_vertical_fov_deg\":" << source.fov
                 << ",\"fx_px\":" << source.focal << ",\"fy_px\":" << source.focal
                 << ",\"cx_px\":" << source.cx << ",\"cy_px\":" << source.cy
                 << ",\"physical_sensor_mm\":null,\"latitude_deg\":" << source.latitude
                 << ",\"longitude_deg\":" << source.longitude << ",\"amsl_m\":" << source.altitude
                 << ",\"commanded_pitch_deg\":90,\"commanded_bank_deg\":0,\"commanded_heading_deg\":0,"
                    "\"observed_pitch_deg\":" << source.pitch << ",\"observed_bank_deg\":" << source.bank
                 << ",\"observed_heading_deg\":" << source.heading << ",\"pose_qpc_100ns\":" << source.poseTime
                 << ",\"frame_qpc_100ns\":" << source.frameTime << ",\"pose_frame_synchronized\":false";
            if (source.captureUtc)
                json << ",\"capture_utc\":" << jsonString(source.captureUtc->text)
                     << ",\"utc_absolute_seconds\":" << std::setprecision(17) << source.utcAbsoluteSeconds
                     << ",\"utc_read_qpc_100ns\":" << source.utcRead100ns
                     << ",\"utc_frame_synchronized\":false" << std::setprecision(15);
            json << '}';
        }
        return json.str() + ']';
    }
    /**
     * @brief Build EXIF for an orthophoto and reference any external source list.
     * @param[in] image Rendered pixels, map grid, coverage classes, and sources.
     * @param[in] projection UTM projection used to derive the map-centre location.
     * @param[in] sourceCompanion Optional companion JSON filename for large provenance.
     * @return Serialized EXIF payload borrowed from the inherited workspace.
     * @throws std::runtime_error If provenance cannot fit or map metadata is invalid.
     */
    Payload buildOrtho(const OrthoImage& image, const MapProjection& projection,
                       const std::string& sourceCompanion = {}) {
        const auto coordinate = projection.inverse(image.map.east, image.map.north);
        build({coordinate.first, coordinate.second, 0, 0}, image.width, image.height);
        check(exif_document_ascii(&document_, EXIF_PRIMARY, EXIF_SOFTWARE, "Pangea / Sensoraia"), "Set map software");
        check(exif_document_ascii(&document_, EXIF_PRIMARY, 0x0110, "Pangea derived map"),
              "Set orthophoto model");
        check(exif_document_ascii(&document_, EXIF_PRIMARY, EXIF_IMAGE_DESCRIPTION,
            "Reference-plane mosaic; relief displacement remains; see coverage metadata"),
            "Set orthophoto description");
        std::ostringstream json;
        json.imbue(std::locale::classic());
        json << std::setprecision(15)
             << "{\"source\":\"pangea/survey\",\"pose\":\"derived_map_center\",\"mode\":" << jsonString(image.mode)
             << ",\"projection\":" << jsonString(image.mode == "simple" ? "flat_plane_mosaic" : "fast_seam_multiband_mosaic")
             << ",\"datum\":\"WGS-84\","
                "\"gps_altitude_meaning\":\"zero_reference_datum_not_camera_or_surface\","
                "\"crs\":\"EPSG:" << image.map.epsg << "\",\"pixel_size_m\":" << image.map.pixelSize
             << ",\"center_easting_m\":" << image.map.east << ",\"center_northing_m\":" << image.map.north
             << ",\"width\":" << image.width << ",\"height\":" << image.height
             << ",\"grid_up_heading_deg\":0,\"valid_fraction\":" << image.validFraction
                 << ",\"confidence_mask\":null,\"surface_heights\":null,"
                "\"absolute_accuracy\":\"simulator_coordinates_not_surveyed\",";
            json << "\"relief_corrected\":false,\"reference_plane_amsl_m\":" << image.referencePlane << ',';
        if (mosaicMode(image.mode))
            json << "\"render_method\":" << jsonString(mosaicRenderMethod(image.seamMethod.empty() ? "graphcut" : image.seamMethod)) << ','
                 << "\"seam_scale\":" << image.seamScale << ",\"blend_bands\":" << image.blendBands
                 << ",\"exposure_compensation\":" << (image.exposureCompensated ? "true" : "false")
                 << ",\"geometry_fraction\":0,\"corroborated_fraction\":0"
                      << ",\"reconstructed_fraction\":" << image.reconstructedFraction
                      << ",\"reconstruction_blend_fraction\":" << image.reconstructionBlendFraction
                      << ",\"observed_colour_fraction\":" << image.validFraction - image.reconstructedFraction
                      << ",\"confidence_classes\":{\"0\":\"unfilled\",\"16\":\"source_derived_reconstruction\","
                          "\"48\":\"observed_reconstruction_transition\",\"64\":\"reference_plane_colour_only\"},";
        const auto sources = sourceMetadata(image.sources);
        if (sources.size() <= 40000) {
            json << "\"sources\":" << sources;
            if (!sourceCompanion.empty()) json << ",\"source_metadata_file\":" << jsonString(sourceCompanion);
        } else {
            if (sourceCompanion.empty()) throw std::runtime_error("Large map source provenance requires a companion JSON");
            json << "\"sources_embedded\":false,\"source_count\":" << image.sources.size()
                 << ",\"source_metadata_file\":" << jsonString(sourceCompanion);
        }
        json << '}';
        const auto text = json.str();
        const unsigned char prefix[] = {'A', 'S', 'C', 'I', 'I', 0, 0, 0};
        if (text.size() > EXIF_MAX_PAYLOAD - sizeof(prefix) - 1)
            throw std::runtime_error("Map provenance exceeds EXIF capacity");
        auto* comment = reserve(EXIF_CAMERA, EXIF_USER_COMMENT, EXIF_UNDEFINED,
                                static_cast<std::uint32_t>(text.size() + 9));
        for (std::size_t offset = 0; offset < sizeof(prefix); ++offset) comment[offset] = prefix[offset];
        for (std::size_t offset = 0; offset < text.size(); ++offset)
            comment[sizeof(prefix) + offset] = static_cast<unsigned char>(text[offset]);
        comment[sizeof(prefix) + text.size()] = 0;
        std::size_t size = 0;
        check(exif_serialize(&document_, payload_.data(), payload_.size(), &size), "Serialize orthophoto EXIF");
        return {payload_.data(), size};
    }
};
}
#endif
