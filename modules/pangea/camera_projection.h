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
 * @file camera_projection.h
 * @brief Map calibrated camera pixels to a local flat ground plane.
 * @details Body and mount rotations are composed before the pinhole ray model.
 * The cached inverse is reused across output pixels; invalid or horizon-facing
 * rays are rejected rather than extrapolated into plausible-looking map points.
 */
#ifndef PANGEA_CAMERA_PROJECTION_H
#define PANGEA_CAMERA_PROJECTION_H

#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>

namespace pangea_survey {
/**
 * @brief Compose body attitude into the local camera-ray coordinate frame.
 * @param[in] roll Body roll in degrees.
 * @param[in] pitch Body pitch in degrees.
 * @param[in] yaw Body yaw in degrees.
 * @return Rotation matrix for the supplied Euler angles.
 */
inline cv::Matx33d cameraRotation(double roll, double pitch, double yaw) {
    constexpr double radians = 3.14159265358979323846 / 180;
    roll *= radians; pitch *= radians; yaw *= radians;
    const cv::Matx33d aroundRoll(1, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, std::sin(roll), std::cos(roll));
    const cv::Matx33d aroundPitch(std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(pitch), 0, std::cos(pitch));
    const cv::Matx33d aroundYaw(std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1);
    return aroundYaw * aroundPitch * aroundRoll;
}

class CameraPlane {
    cv::Matx33d rays_, inverseRays_;
public:
    /**
     * @brief Cache forward and inverse pinhole transforms for one camera pose.
     * @param[in] fx Horizontal focal length in pixels.
     * @param[in] fy Vertical focal length in pixels.
     * @param[in] cx Principal-point x coordinate in pixels.
     * @param[in] cy Principal-point y coordinate in pixels.
     * @param[in] roll Body roll in degrees.
     * @param[in] pitch Body pitch in degrees.
     * @param[in] yaw Body yaw in degrees.
     * @param[in] mountRoll Camera mount roll in degrees.
     * @param[in] mountPitch Camera mount pitch in degrees.
     * @param[in] mountYaw Camera mount yaw in degrees.
     * @throws std::runtime_error If intrinsics or angles are nonfinite, or focal length is invalid.
     */
    CameraPlane(double fx, double fy, double cx, double cy, double roll, double pitch, double yaw,
                double mountRoll = 0, double mountPitch = 0, double mountYaw = 0) {
        for (const double value : {fx, fy, cx, cy, roll, pitch, yaw, mountRoll, mountPitch, mountYaw})
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite camera geometry");
        if (fx <= 0 || fy <= 0) throw std::runtime_error("Invalid camera focal length");
        rays_ = cameraRotation(roll, pitch, yaw) * cameraRotation(mountRoll, mountPitch, mountYaw) *
            cv::Matx33d(0, -1 / fy, cy / fy, 1 / fx, 0, -cx / fx, 0, 0, 1);
        inverseRays_ = rays_.inv();
    }

    /**
     * @brief Intersect one delivered-image pixel ray with a horizontal ground plane.
     * @param[in] column Image column in pixels.
     * @param[in] row Image row in pixels.
     * @param[in] height Camera-to-plane distance in metres.
     * @return Local ground point in metres.
     * @throws std::runtime_error If the ray is invalid or too close to the horizon.
     */
    cv::Point2d ground(double column, double row, double height) const {
        const auto ray = rays_ * cv::Vec3d(column, row, 1);
        const double length = cv::norm(ray);
        if (!std::isfinite(height) || height <= 5 || !std::isfinite(length) || length <= 0 || ray[2] / length < .2)
            throw std::runtime_error("Camera footprint reaches horizon or invalid ground plane");
        return {height * ray[1] / ray[2], height * ray[0] / ray[2]};
    }

    /**
     * @brief Return a homography from local ground coordinates to image pixels.
     * @param[in] height Camera-to-plane distance in metres.
     * @return 3x3 ground-to-pixel projective transform.
     * @throws std::runtime_error If the plane height is invalid.
     */
    cv::Matx33d groundToPixel(double height) const {
        if (!std::isfinite(height) || height <= 5) throw std::runtime_error("Invalid camera ground distance");
        return inverseRays_ * cv::Matx33d(0, 1, 0, 1, 0, 0, 0, 0, height);
    }

    /**
     * @brief Shift the principal point after removing equal borders from the image.
     * @param[in] inset Number of pixels removed from each edge.
     */
    void crop(int inset) {
        rays_ = rays_ * cv::Matx33d(1, 0, inset, 0, 1, inset, 0, 0, 1);
        inverseRays_ = cv::Matx33d(1, 0, -inset, 0, 1, -inset, 0, 0, 1) * inverseRays_;
    }
};
}
#endif