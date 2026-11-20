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
 * @file map_pipeline.h
 * @brief Orchestrate orthophoto rendering, evidence, and final publication.
 * @details Each output directory begins with an incomplete marker and is marked
 * complete only after all required images and manifests have been finalized.
 * Diagnostic runs intentionally retain incomplete status because they inspect
 * imagery/coverage but do not pass the production publication gate.
 */
#ifndef PANGEA_MAP_PIPELINE_H
#define PANGEA_MAP_PIPELINE_H

#include "binary_output.h"
/**
 * @brief Render orthophoto tiles and publish their coverage/provenance manifest.
 * @param[in] config Validated map settings and colour-coverage threshold.
 * @param[in] csv Source CSV, normalized by the caller when required.
 * @param[in] diagnostic Render inspectable evidence without publishing a complete dataset.
 * @param[in] progress Optional caller-owned progress reporter.
 * @param[in] first Starting progress percentage for a larger pipeline.
 * @return Zero after successful rendering and finalization.
 * @throws std::runtime_error If rendering, coverage policy, or publication fails.
 * @note Coverage acceptance is not surveyed-accuracy certification.
 */
static int orthorectify(const pangea_survey::MapConfig& config, const fs::path& csv, bool diagnostic = false,
                       pangea_survey::MapProgress* progress = nullptr, double first = 0) {
    std::unique_ptr<pangea_survey::MapProgress> ownedProgress;
    if (!progress) {
        ownedProgress = std::make_unique<pangea_survey::MapProgress>(diagnostic ? "Diagnostic progress" : "Map progress");
        progress = ownedProgress.get();
    }
    const double renderStart = first + (99 - first) * .02;
    progress->stage("Loading sources and surface", first, renderStart);
    pangea_survey::Orthorectifier processor(config, csv);
    const auto directory = config.output / ("orthomap_" + sessionName());
    fs::create_directories(directory);
    std::cout << "Pangea " << config.mode << ": " << processor.columns() << " x " << processor.rows()
              << " final tiles, zero tile overlap, " << config.mapPixelSize
              << " m/projected pixel. Output: " << directory.u8string() << '\n';
    JpegEncoder encoder;
    pangea_survey::MapExif exif;
    const pangea_survey::MapProjection projection((config.start.lat + config.end.lat) / 2,
                                           (config.start.lon + config.end.lon) / 2);
    std::ofstream manifest(directory / "orthophotos.csv");
    manifest << "filename,row,column,epsg,center_easting_m,center_northing_m,pixel_size_m,width,height,"
                "valid_fraction,projection,source_csv\n";
    if (!manifest) throw std::runtime_error("Cannot create orthophoto manifest");
    const auto incomplete = "{\"complete\":false,\"mode\":" + pangea_survey::PhotoExif::jsonString(config.mode) + "}";
    // Keep the incomplete marker until every required tile and final manifest exist.
    saveBinary(directory / "dataset.incomplete.json", incomplete.data(), incomplete.size());
    progress->stage(diagnostic ? "Rendering diagnostic evidence" : "Creating and saving map tiles", renderStart, 99);
    int effectiveSeamScale = 0;
    double effectiveReferencePlane = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < processor.size(); ++i) {
        const auto detail = "tile " + std::to_string(i + 1) + "/" + std::to_string(processor.size());
        progress->detail(detail);
        auto image = processor.render(i, isStopping, [&](double fraction) {
            progress->update((i + .95 * fraction) / processor.size(), detail);
        });
        effectiveSeamScale = image.seamScale;
        effectiveReferencePlane = image.referencePlane;
        std::ostringstream basename;
        basename << "ortho_r" << std::setfill('0') << std::setw(5) << i / processor.columns()
                 << "_c" << std::setw(5) << i % processor.columns();
        const auto base = directory / basename.str();
        std::ostringstream geometry;
        geometry.imbue(std::locale::classic());
        geometry << std::setprecision(15) << "{\"epsg\":" << image.map.epsg
                 << ",\"center_easting_m\":" << image.map.east << ",\"center_northing_m\":" << image.map.north
                 << ",\"pixel_size_m\":" << image.map.pixelSize << ",\"width\":" << image.width
                 << ",\"height\":" << image.height << ",\"valid_fraction\":" << image.validFraction
                 << ",\"mode\":" << pangea_survey::PhotoExif::jsonString(config.mode)
                 << ",\"height_encoding\":null"
                 << ",\"relief_corrected\":false,\"reference_plane_amsl_m\":" << image.referencePlane
                            << ",\"confidence_classes\":{\"0\":\"no_colour\",\"16\":\"source_derived_reconstruction\","
                                "\"48\":\"observed_reconstruction_transition\",\"64\":\"flat_plane_colour_only\"}";
        if (pangea_survey::mosaicMode(config.mode))
            geometry << ",\"render_method\":" << pangea_survey::PhotoExif::jsonString(pangea_survey::mosaicRenderMethod(image.seamMethod))
                     << ",\"seam_scale\":" << image.seamScale
                     << ",\"blend_bands\":" << image.blendBands
                     << ",\"reconstructed_fraction\":" << image.reconstructedFraction
                     << ",\"reconstruction_blend_fraction\":" << image.reconstructionBlendFraction
                     << ",\"observed_colour_fraction\":" << image.validFraction - image.reconstructedFraction
                     << ",\"exposure_compensation\":" << (image.exposureCompensated ? "true" : "false")
                     << ",\"geometry_fraction\":0,\"corroborated_fraction\":0";
        geometry << '}';
        const auto geometryText = geometry.str();
        saveBinary(base.string() + ".geometry.json", geometryText.data(), geometryText.size());
        const fs::path sourceCompanion = base.string() + ".sources.json";
        const auto sourcesText = "{\"source_csv\":" + pangea_survey::PhotoExif::jsonString(csv.u8string()) +
                                 ",\"sources\":" + pangea_survey::MapExif::sourceMetadata(image.sources) + "}";
        saveBinary(sourceCompanion, sourcesText.data(), sourcesText.size());
        if (diagnostic) {
            const fs::path preview = base.string() + ".diagnostic.jpg";
            const auto jpeg = encoder.encode(image.bgra.data(), image.width, image.height,
                                             image.width * 4, static_cast<int>(config.quality));
            if (pangea_survey::mosaicMode(config.mode)) {
                const auto metadata = exif.buildOrtho(image, projection, sourceCompanion.filename().u8string());
                pangea_survey::MapWriter::writeMap(preview, image.map, image.width, image.height, false);
                JpegEncoder::save(jpeg, preview, metadata.data, metadata.size);
            } else JpegEncoder::save(jpeg, preview);
            // Diagnostic artifacts help inspect coverage but are never a completed publication.
            std::cout << "Diagnostic only: unapproved image/coverage evidence saved; no final tile published.\n";
            progress->update(static_cast<double>(i + 1) / processor.size(), detail);
            continue;
        }
        // Refuse a plausible-looking output tile when unsupported pixels exceed policy.
        if (image.validFraction < config.minCoverage)
            throw std::runtime_error("Map colour coverage is below quality.min_colour_coverage; final JPEG not published. "
                "Inspect saved coverage evidence; acquire additional source photos or reduce the requested bounds.");
        const fs::path path = base.string() + ".jpg";
        auto world = path; world.replace_extension(".jgw");
        const fs::path aux = path.string() + ".aux.xml";
        try {
            pangea_survey::MapWriter::writeMap(path, image.map, image.width, image.height, false);
            const auto metadata = exif.buildOrtho(image, projection, sourceCompanion.filename().u8string());
            const auto jpeg = encoder.encode(image.bgra.data(), image.width, image.height,
                                             image.width * 4, static_cast<int>(config.quality));
            JpegEncoder::save(jpeg, path, metadata.data, metadata.size);
        } catch (...) {
            for (const auto& file : {world, aux}) {
                std::error_code error;
                fs::remove(file, error);
                if (error) std::cerr << "WARNING: Cannot clean unpublished orthophoto metadata: " << error.message() << '\n';
            }
            throw;
        }
        manifest << path.filename().u8string() << ',' << i / processor.columns() << ',' << i % processor.columns()
                 << ',' << image.map.epsg << ',' << std::setprecision(15) << image.map.east
                 << ',' << image.map.north << ',' << image.map.pixelSize << ',' << image.width
                 << ',' << image.height                  << ',' << image.validFraction << ','
                 << (config.mode == "simple" ? "flat_plane_mosaic" : "fast_seam_multiband_mosaic") << ",\"";
        for (const char ch : csv.u8string()) { if (ch == '"') manifest << '"'; manifest << ch; }
        manifest << "\"\n";
        manifest.flush();
        if (!manifest) throw std::runtime_error("Cannot write orthophoto manifest");
        std::cout << "Saved " << config.mode << " map tile " << path.filename().u8string() << '\n';
        progress->update(static_cast<double>(i + 1) / processor.size(), detail);
    }
    progress->stage("Finalizing dataset metadata", 99, 99.9);
    manifest.close();
    if (!manifest) throw std::runtime_error("Cannot finalize orthophoto manifest");
    std::ostringstream dataset;
    dataset.imbue(std::locale::classic());
    dataset << std::setprecision(15) << "{\"complete\":" << (diagnostic ? "false" : "true")
            << ",\"application\":\"pangea\",\"mode\":" << pangea_survey::PhotoExif::jsonString(config.mode)
            << ",\"blend\":" << pangea_survey::PhotoExif::jsonString(pangea_survey::mosaicMode(config.mode) ? "multiband" : config.blend)
            << ",\"min_colour_coverage\":" << config.minCoverage
                << ",\"relief_corrected\":false"
                << ",\"reference_plane_amsl_m\":" << effectiveReferencePlane;
            dataset << ",\"mosaic_seam_scale\":" << effectiveSeamScale
                << ",\"mosaic_blend_bands\":" << (pangea_survey::mosaicMode(config.mode) ? config.mosaicBlendBands : 0)
                << ",\"mosaic_exposure_compensation\":" << (pangea_survey::mosaicMode(config.mode) && config.mosaicExposure ? "true" : "false")
            << ",\"mosaic_seam_method\":" << pangea_survey::PhotoExif::jsonString(config.mosaicSeamMethod)
            << ",\"mosaic_overview_pixels\":" << config.mosaicOverviewPixels
            << ",\"mosaic_photo_cache_mib\":" << config.mosaicPhotoCacheMiB
            << ",\"mosaic_overview_source_max_side\":" << config.mosaicOverviewSourceMaxSide
            << ",\"mosaic_fill_gaps\":" << (config.mosaicFillGaps ? "true" : "false")
            << ",\"gap_fill_method\":" << pangea_survey::PhotoExif::jsonString(config.mosaicFillGaps ? "global_source_push_pull" : "none")
            << ",\"mosaic_threads\":" << config.mosaicThreads
            << ",\"render_method\":" << pangea_survey::PhotoExif::jsonString(pangea_survey::mosaicMode(config.mode) ?
                pangea_survey::mosaicRenderMethod(config.mosaicSeamMethod) : "flat_plane_mosaic")
            << ",\"original_source_csv\":" << pangea_survey::PhotoExif::jsonString(config.originalSourceCsv.u8string())
            << ",\"diagnostic\":" << (diagnostic ? "true" : "false")
            << ",\"accuracy_certified\":false,\"columns\":" << processor.columns()
            << ",\"rows\":" << processor.rows() << ",\"pixel_size_m\":" << config.mapPixelSize
            << ",\"tile_width\":" << config.jpegWidth << ",\"tile_height\":" << config.jpegHeight
            << ",\"output_overlap_pixels\":0,\"source_csv\":" << pangea_survey::PhotoExif::jsonString(csv.u8string())
            << ",\"requested_startup_utc\":"
            << (config.startupUtc ? pangea_survey::PhotoExif::jsonString(config.startupUtc->text) : "null")
            << ",\"requested_weather_preset\":" << pangea_survey::PhotoExif::jsonString(config.weatherPreset)
            << ",\"weather_verification\":"
            << (config.weatherReadback.empty() ? "null" : "\"SDK_FlightSave_readback_before_capture\"")
            << ",\"weather_readback_file\":"
            << (config.weatherReadback.empty() ? "null" : pangea_survey::PhotoExif::jsonString(config.weatherReadback.u8string()))
            << ",\"capture_utc\":"
            << (processor.captureUtc() ? pangea_survey::PhotoExif::jsonString(processor.captureUtc()->text) : "null")
            << ",\"simulation_time_continues\":" << (processor.captureUtc() ? "false" : "null")
            << ",\"utc_hold_method\":"
            << (processor.captureUtc() ? "\"repeated_simconnect_clock_resets\"" : "null") << "}";
    const auto datasetText = dataset.str();
    saveBinary(directory / "dataset.json", datasetText.data(), datasetText.size());
    if (!fs::remove(directory / "dataset.incomplete.json"))
        throw std::runtime_error("Cannot remove incomplete dataset marker");
    std::cout << (diagnostic ? "Diagnostic render completed; not a completed map dataset.\n" :
        "Map dataset completed; coverage is not surveyed accuracy certification.\n");
    progress->finish();
    return 0;
}
#endif
