#include "armor_detector/light_corner_corrector.hpp"

#include <numeric>

namespace rm_auto_aim {

void LightCornerCorrector::correctCorners(Armor &armor, const cv::Mat &gray_img) {
  // If the width of the light is too small, the correction is not performed
  constexpr int PASS_OPTIMIZE_WIDTH = 3;

  if (armor.left_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light

    SymmetryAxis left_axis = findSymmetryAxis(gray_img, armor.left_light);
    armor.left_light.center = left_axis.centroid;
    armor.left_light.axis = left_axis.direction;
   std::cout<<"left_light.center"<<left_axis.centroid<<std::endl;

    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.left_light, left_axis, "top"); t.x > 0) {
      armor.left_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.left_light, left_axis, "bottom"); b.x > 0) {
      armor.left_light.bottom = b;
    }
  }

  if (armor.right_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light
    SymmetryAxis right_axis = findSymmetryAxis(gray_img, armor.right_light);
    armor.right_light.center = right_axis.centroid;
    armor.right_light.axis = right_axis.direction;
    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.right_light, right_axis, "top"); t.x > 0) {
      armor.right_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.right_light, right_axis, "bottom"); b.x > 0) {
      armor.right_light.bottom = b;
    }
  }
}

SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, const Light &light) {
  // constexpr float MAX_BRIGHTNESS = 25;
  // constexpr float SCALE = 0.07;

  // // Scale the bounding box
  // cv::Rect light_box = light.boundingRect();
  // light_box.x -= static_cast<int>(light_box.width * SCALE);
  // light_box.y -= static_cast<int>(light_box.height * SCALE);
  // light_box.width += static_cast<int>(light_box.width * SCALE * 2);
  // light_box.height += static_cast<int>(light_box.height * SCALE * 2);

  // // 确保坐标在图像范围内
  // light_box.x = std::max(0, light_box.x);
  // light_box.y = std::max(0, light_box.y);
  // // 确保宽度和高度不超出剩余空间且≥1
  // light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  // light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);
  // light_box.width = std::max(1, light_box.width);
  // light_box.height = std::max(1, light_box.height);
  //           std::cout<<"555566666"<<std::endl;

  //二次检查避免计算后越界
  // if (light_box.x + light_box.width > gray_img.cols || 
  //     light_box.y + light_box.height > gray_img.rows) {
  //   return SymmetryAxis{}; // 返回默认值或抛出异常
  // }



  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.07f;

  // 防御层1：原始包围盒合法性检查
  cv::Rect light_box = light.boundingRect();
  if (light_box.empty() || 
      light_box.x >= gray_img.cols || 
      light_box.y >= gray_img.rows) {
    return SymmetryAxis{};
  }

  // 防御层2：智能动态缩放
  auto calcExpansion = [](int pos, int length, int img_max, float scale) -> int {
    int max_expand = static_cast<int>(length * scale);
    return std::min(std::min(pos, max_expand),          // 向左/上可扩展空间
                    std::min(img_max - (pos + length),  // 向右/下可扩展空间
                             max_expand));
  };

  int expand_left = calcExpansion(light_box.x, light_box.width, gray_img.cols, SCALE);
  int expand_right = calcExpansion(gray_img.cols - (light_box.x + light_box.width), 
                                  light_box.width, gray_img.cols, SCALE);
  int expand_top = calcExpansion(light_box.y, light_box.height, gray_img.rows, SCALE);
  int expand_bottom = calcExpansion(gray_img.rows - (light_box.y + light_box.height),
                                   light_box.height, gray_img.rows, SCALE);

  // 应用缩放
  light_box.x -= expand_left;
  light_box.y -= expand_top;
  light_box.width += expand_left + expand_right;
  light_box.height += expand_top + expand_bottom;

  // 防御层3：边界硬钳制（核心防御）
  auto hardClamp = [](cv::Rect& rect, const cv::Size& img_size) {
    rect.x = std::clamp(rect.x, 0, img_size.width - 1);
    rect.y = std::clamp(rect.y, 0, img_size.height - 1);
    rect.width = std::clamp(rect.width, 1, img_size.width - rect.x);
    rect.height = std::clamp(rect.height, 1, img_size.height - rect.y);
  };
  hardClamp(light_box, gray_img.size());

  // 防御层4：最终合法性验证（终极保障）
  const cv::Rect img_rect(0, 0, gray_img.cols, gray_img.rows);
  light_box = light_box & img_rect; // OpenCV矩形交集运算
  if (light_box.empty()) {
    light_box = light.boundingRect(); // 回退原始包围盒
    light_box = light_box & img_rect; // 再次确保合法性
    if (light_box.empty()) return SymmetryAxis{};
  }

  // 获取ROI
  cv::Mat roi = gray_img(light_box);
            std::cout<<"7777777"<<std::endl;
  float mean_val = cv::mean(roi)[0];

          std::cout<<"33333333333"<<std::endl;
  roi.convertTo(roi, CV_32F);


  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);
        std::cout<<"211111222"<<std::endl;
  // Calculate the centroid
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                         cv::Point2f(light_box.x, light_box.y);
    std::cout<<"center"<<centroid<<std::endl;
  // Initialize the PointCloud
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // PCA (Principal Component Analysis)
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get the symmetry axis
  cv::Point2f axis =
    cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

  // Normalize the axis
  axis = axis / cv::norm(axis);

  if (axis.y > 0) {
    axis = -axis;
  }

  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const Light &light,
                                             const SymmetryAxis &axis,
                                             std::string order) {
  constexpr float START = 0.8 / 2;
  constexpr float END = 1.2 / 2;

  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };


  int oper = order == "top" ? 1 : -1;
  float L = light.length;
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  // Select multiple corner candidates and take the average as the final corner
  int n = light.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;
    // Search along the symmetry axis to find the corner that has the maximum brightness difference
    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      float brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }

      prev = cur;
    }

    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }
  if (!candidates.empty()) {
    cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }

  return cv::Point2f(-1, -1);
}

}  // namespace fyt::auto_aim
