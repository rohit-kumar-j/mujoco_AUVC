#include "AR_SRC/TagDetector.h"
#include <mujoco/mujoco.h>
#include <opencv2/opencv.hpp>

#ifndef MUJOCO_SIMULATE_SIMULATE_H_
#define MUJOCO_SIMULATE_SIMULATE_H_
struct camData_{ // Utility struct to hold opencv: images
  cv::Mat *image;
  cv::Mat *flipped;
  cv::Mat *image_gray;
}; typedef struct camData_ camData;



#endif // MUJOCO_SIMULATE_SIMULATE_H_
#ifndef AR_TAGS
#define AR_TAGS

#define maxPoints 100

struct ExtractedPoints_{
    int nPoints;
    float points[8 * maxPoints];
};
typedef struct ExtractedPoints_ ExtractedPoints_;

namespace auvc{

int processImage(cv::Mat& image, cv::Mat& image_gray, cv::Mat& flipped, float pts[8], AprilTags::TagDetector* m_tagDetector);
// void processImage(camData *cvData, auvcData* rawColorData);

}
#endif //AR_TAGS
