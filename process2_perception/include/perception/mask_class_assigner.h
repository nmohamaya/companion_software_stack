// process2_perception/include/perception/mask_class_assigner.h
// Assigns detector class labels to SAM masks via bbox IoU.
// Unmatched masks receive GEOMETRIC_OBSTACLE (class-agnostic).
#pragma once

#include "hal/iinference_backend.h"
#include "perception/detector_class_maps.h"
#include "perception/types.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace drone::perception {

struct MaskAssignment {
    hal::InferenceDetection mask_detection;
    ObjectClass             assigned_class{ObjectClass::GEOMETRIC_OBSTACLE};
    float                   assignment_iou{0.0f};
    int                     detector_class_id{-1};
};

class MaskClassAssigner {
public:
    explicit MaskClassAssigner(float iou_threshold = 0.5f) : iou_threshold_(iou_threshold) {}

    // Greedy IoU-based assignment: detector bboxes sorted by confidence desc,
    // each matched to highest-IoU unmatched mask. Unmatched masks keep
    // GEOMETRIC_OBSTACLE. Each detector bbox matches at most one mask.
    [[nodiscard]] std::vector<MaskAssignment> assign(
        const std::vector<hal::InferenceDetection>& sam_masks,
        const std::vector<hal::InferenceDetection>& detector_outputs) const {

        std::vector<MaskAssignment> result;
        result.reserve(sam_masks.size());
        for (const auto& mask : sam_masks) {
            MaskAssignment ma;
            ma.mask_detection = mask;
            result.push_back(std::move(ma));
        }

        if (detector_outputs.empty() || sam_masks.empty()) {
            return result;
        }

        // Sort detectors by confidence descending (greedy — highest confidence first)
        std::vector<size_t> det_order(detector_outputs.size());
        for (size_t i = 0; i < det_order.size(); ++i) det_order[i] = i;
        std::sort(det_order.begin(), det_order.end(), [&](size_t a, size_t b) {
            return detector_outputs[a].confidence > detector_outputs[b].confidence;
        });

        std::vector<bool> mask_matched(sam_masks.size(), false);

        for (size_t di : det_order) {
            const auto& det      = detector_outputs[di];
            float       best_iou = 0.0f;
            size_t      best_idx = 0;
            bool        found    = false;

            for (size_t mi = 0; mi < sam_masks.size(); ++mi) {
                if (mask_matched[mi]) continue;
                float iou = compute_bbox_iou(det.bbox, sam_masks[mi].bbox);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_idx = mi;
                    found    = true;
                }
            }

            if (found && best_iou >= iou_threshold_) {
                mask_matched[best_idx]             = true;
                result[best_idx].assigned_class    = coco_to_object_class(det.class_id);
                result[best_idx].assignment_iou    = best_iou;
                result[best_idx].detector_class_id = det.class_id;
            }
        }

        return result;
    }

    static float compute_bbox_iou(const hal::BoundingBox2D& a, const hal::BoundingBox2D& b) {
        const float x1 = std::max(a.x, b.x);
        const float y1 = std::max(a.y, b.y);
        const float x2 = std::min(a.x + a.w, b.x + b.w);
        const float y2 = std::min(a.y + a.h, b.y + b.h);

        if (x2 <= x1 || y2 <= y1) return 0.0f;

        const float intersection = (x2 - x1) * (y2 - y1);
        const float area_a       = a.w * a.h;
        const float area_b       = b.w * b.h;
        const float union_area   = area_a + area_b - intersection;

        if (union_area <= 0.0f) return 0.0f;
        return intersection / union_area;
    }

private:
    float iou_threshold_;
};

}  // namespace drone::perception
