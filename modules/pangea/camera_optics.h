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
 * @file camera_optics.h
 * @brief Optical profiles and EXIF settings for simulated survey cameras.
 * @details The Pi AI profile is a deterministic rendering model, not a
 * measured calibration. Its crop, field of view, distortion, and channel scales
 * must stay coupled so the saved image and intrinsics describe the same pixels.
 */
#ifndef PANGEA_CAMERA_OPTICS_H
#define PANGEA_CAMERA_OPTICS_H

#include "image_framing.h"
#include <cstdint>
#include <string>

namespace pangea_survey {
/**
 * @brief User-supplied exposure labels written to simulated-photo EXIF.
 * @details These values describe the configured emulation only; they are not
 * sensor readback or evidence of a physical exposure.
 */
struct CameraExifSettings {
    double shutterSeconds = 0.001;
    std::uint16_t iso = 100;
    std::string hardwareIdentifier;

    /** @brief Reject values that cannot be represented as bounded EXIF metadata. */
    void validate() const {
        if (!std::isfinite(shutterSeconds) || shutterSeconds < 0.000001 || shutterSeconds > 30)
            throw std::runtime_error("camera.shutter_speed_seconds must be between 0.000001 and 30");
        if (iso == 0)
            throw std::runtime_error("camera.iso must be an integer between 1 and 65535");
        if (hardwareIdentifier.size() > 64 || hardwareIdentifier.find_first_not_of(
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._:-") != std::string::npos)
            throw std::runtime_error("camera.hardware_identifier must contain 1-64 letters, digits, '.', '_', ':' or '-'");
    }
};

/** @brief Engineering lens-distortion and lateral colour-scale parameters. */
struct LensEffects {
    double k1 = -.03, k2 = .005;
    double blueScale = .9996, redScale = 1.0004;
};
/**
 * @brief Nominal Pi AI Camera geometry used by the simulator rendering profile.
 * @details Fixed dimensions and focal model define the intrinsic matrix;
 * overscan keeps rotated/distorted output inside the final frame without
 * changing those nominal intrinsics.
 */
struct PiAiOptics {
    static constexpr int width = 2028, height = 1520;
    static constexpr double focalMm = 4.74, aperture = 1.79;
    static constexpr double clearDayShutterSeconds = 0.0001;
    static constexpr double sensorWidthMm = 4056 * 0.00155;
    static constexpr double sensorHeightMm = 3040 * 0.00155;
    static constexpr double radiansToDegrees = 57.295779513082320877;
    static constexpr double focalPixels = focalMm / (2 * 0.00155);
    static constexpr double k1 = -0.03, k2 = 0.005;
    static constexpr double blueScale = 0.9996, greenScale = 1, redScale = 1.0004;
    static constexpr double overscan = 1.04;
    /**
     * @brief Compute a centered source crop matching the nominal sensor frame.
     * @param[in] sourceWidth Rendered source width in pixels.
     * @param[in] sourceHeight Rendered source height in pixels.
     * @param[in] inset Pixels excluded at each source edge.
     * @return Crop geometry and its shared horizontal/vertical sampling scale.
     */
    static ImageFraming::Crop crop(int sourceWidth, int sourceHeight, int inset) {
        auto result = ImageFraming::crop(sourceWidth, sourceHeight, width, height, inset);
        result.scale /= overscan;
        result.width = width * result.scale;
        result.height = height * result.scale;
        result.x = (sourceWidth - result.width) / 2;
        result.y = (sourceHeight - result.height) / 2;
        return result;
    }
    /** @brief Return nominal vertical field of view in degrees. */
    static double verticalFov() {
        return 2 * std::atan(height / (2 * focalPixels)) * radiansToDegrees;
    }
    /** @brief Return nominal horizontal field of view in degrees. */
    static double horizontalFov() {
        return 2 * std::atan(width / (2 * focalPixels)) * radiansToDegrees;
    }
    /**
     * @brief Find the simulator vertical FOV that yields this captured viewport.
     * @param[in] sourceWidth Rendered viewport width in pixels.
     * @param[in] sourceHeight Rendered viewport height in pixels.
     * @param[in] inset Excluded source-edge pixels used by the crop.
     * @return Vertical field of view in degrees.
     * @throws std::runtime_error If the viewport needs an unsupported FOV.
     */
    static double viewportFov(int sourceWidth, int sourceHeight, int inset) {
        const auto framing = crop(sourceWidth, sourceHeight, inset);
        const double result = 2 * std::atan(sourceHeight / (2 * focalPixels * framing.scale)) * radiansToDegrees;
        if (result < 10 || result > 120)
            throw std::runtime_error("Viewport/inset cannot represent the Pi AI lens within SDK FOV limits");
        return result;
    }
};
}

#endif
