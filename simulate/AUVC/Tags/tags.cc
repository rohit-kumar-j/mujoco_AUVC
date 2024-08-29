#include <mujoco/mujoco.h>
#include <vector>

#include "tags.h"
#include "AR_SRC/Gaussian.h"
#include "AR_SRC/FloatImage.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "AR_SRC/Edge.h"
#include "AR_SRC/FloatImage.h"
#include "AR_SRC/GLine2D.h"
#include "AR_SRC/GLineSegment2D.h"
#include "AR_SRC/Gaussian.h"
#include "AR_SRC/GrayModel.h"
#include "AR_SRC/Gridder.h"
#include "AR_SRC/Homography33.h"
#include "AR_SRC/MathUtil.h"
#include "AR_SRC/Quad.h"
#include "AR_SRC/Segment.h"
#include "AR_SRC/TagFamily.h"
#include "AR_SRC/UnionFindSimple.h"
#include "AR_SRC/XYWeight.h"

#include "AR_SRC/Tag36h11.h"

#include "AR_SRC/TagDetector.h"

int auvc::processImage(cv::Mat& image, cv::Mat& image_gray, cv::Mat& flipped, float pts[8], AprilTags::TagDetector* m_tagDetector){
  cv::cvtColor(image, flipped, cv::COLOR_RGB2GRAY);
  printf("Locked 2\n");
  cv::flip(flipped, image_gray, 0);

  // printf("rows %d : cols %du\n",image.rows, image.cols);
  // printf("img.step/img.elemSize() : %zu\n",image.step/image.elemSize());
  // cv::imshow("window", image_gray);
  // cv::waitKey(0);

  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  printf("Number of Tags detected = %zu\n", detections.size());

  float points_outline[8] = {0};

  for (int i=0; i<detections.size(); i++) {
    // TODO: Get Detection Angle, position etc
    printf("Id: %d, Hamming %d\n",detections[i].id, detections[i].hammingDistance);
    detections[i].draw2(image,points_outline);
  }

  for (int i=0; i<detections.size(); i++) {
    pts[i] = points_outline[i];
  }

  for (int i=0; i<8; i++) {
    printf("pts: %f\n",points_outline[i]);
  }

  delete m_tagDetector;

  return detections.size();

}
