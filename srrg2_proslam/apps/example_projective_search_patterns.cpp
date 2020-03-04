#include <iostream>
#include <srrg_image/image.h>

using namespace srrg2_core;

void drawSearchRegionSquare(const int16_t radius_,
                            const int16_t row_,
                            const int16_t col_,
                            const int16_t color_,
                            cv::Mat& canvas_) {
  for (int16_t r = row_ - radius_; r < row_ + radius_ + 1; ++r) {
    for (int16_t c = col_ - radius_; c < col_ + radius_ + 1; ++c) {
      canvas_.at<uchar>(r, c) = color_;
    }
  }
}

void drawSearchRegionCircle(const int16_t radius_,
                            const int16_t row_,
                            const int16_t col_,
                            const int16_t color_,
                            cv::Mat& canvas_) {
  const int16_t radius_squared = radius_ * radius_;
  for (int16_t r = row_ - radius_; r < row_ + radius_ + 1; ++r) {
    const int16_t height = r - row_;
    const int16_t width  = std::sqrt(radius_squared - height * height);
    for (int16_t c = col_ - width; c < col_ + width + 1; ++c) {
      canvas_.at<uchar>(r, c) = color_;
    }
  }
}

void drawSearchRegionRhombus(const int16_t radius_,
                             const int16_t row_,
                             const int16_t col_,
                             const int16_t color_,
                             cv::Mat& canvas_) {
  const int16_t row_start = row_ - radius_;
  const int16_t row_stop  = row_ + radius_;
  for (int16_t r = row_start - radius_; r < row_stop; ++r) {
    int16_t width = r - row_start;
    if (width > radius_) {
      width = row_stop - r;
    }
    for (int16_t c = col_ - width; c < col_ + width + 1; ++c) {
      canvas_.at<uchar>(r, c) = color_;
    }
  }
}

int main(int argc_, char** argv_) {
  // ds configuration
  const int16_t image_rows = 500;
  const int16_t image_cols = 500;

  // ds canvas
  cv::Mat image(image_rows, image_cols, CV_8UC1, cv::Scalar(0));

  // ds sample a bunch of search regions: rectangle
  drawSearchRegionSquare(100, 150, 150, 75, image);
  drawSearchRegionSquare(50, 325, 325, 75, image);
  drawSearchRegionSquare(10, 410, 410, 75, image);
  drawSearchRegionSquare(5, 450, 450, 75, image);

  // ds rectangle
  drawSearchRegionCircle(100, 150, 150, 150, image);
  drawSearchRegionCircle(50, 325, 325, 150, image);
  drawSearchRegionCircle(10, 410, 410, 150, image);
  drawSearchRegionCircle(5, 450, 450, 150, image);

  // ds rhombus
  drawSearchRegionRhombus(100, 150, 150, 255, image);
  drawSearchRegionRhombus(50, 325, 325, 255, image);
  drawSearchRegionRhombus(10, 410, 410, 255, image);
  drawSearchRegionRhombus(5, 450, 450, 255, image);

  // ds show canvas
  cv::imshow("search patterns: RECTANGLE | PRISM | CIRCLE", image);
  cv::waitKey(0);
  return 0;
}
