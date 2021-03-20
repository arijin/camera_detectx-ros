#include <image_rectify.h>
#include <iostream>

bool cache_ = false;
cv::Mat_<double> D_;
cv::Matx33d R_;
cv::Matx33d K_;
cv::Matx34d P_;
cv::Size dst_size_;

inline double fx() { return P_(0, 0); }
inline double fy() { return P_(1, 1); }
inline double cx() { return P_(0, 2); }
inline double cy() { return P_(1, 2); }
inline double Tx() { return P_(0, 3); }
inline double Ty() { return P_(1, 3); }
bool initialized() { return cache_; }

void infoInit(std::string camera_info_dir)
{
    cache_ = false;

    cv::FileStorage fss(camera_info_dir, cv::FileStorage::READ);
    if (!fss.isOpened())
    {
        std::cout << "Invalid camera calibration filename." << std::endl;
        return;
    }
    
    int width=fss["image_width"];
    int height=fss["image_height"];
    cv::Size dst_size(width, height);  // 从左上角按照dst_size大小切
    dst_size_ = dst_size;

    cv::Mat K_temp,P_temp, R_temp,D_temp;
    fss["camera_matrix"] >> K_temp;
    fss["distortion_coefficients"] >> D_temp;
    // R_temp = cv::Mat_<double>::eye(3, 3);
    fss["rectification_matrix"] >> R_temp;
    fss["projection_matrix"] >> P_temp;
    fss.release();
    
    K_ = K_temp.clone();
    D_ = D_temp.clone();
    R_ = R_temp.clone();
    P_ = P_temp.clone();
    
    cache_ = true;
    return;
}

cv::Point2d project3dToPixel(const cv::Point3d &xyz)
{
    assert(initialized());
    assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane
    cv::Point2d uv_rect;
    uv_rect.x = (fx() * xyz.x + Tx()) / xyz.z + cx();
    uv_rect.y = (fy() * xyz.y + Ty()) / xyz.z + cy();
    return uv_rect;
}

cv::Point3d projectPixelTo3dRay(const cv::Point2d &uv_rect)
{
    assert(initialized());

    cv::Point3d ray;
    ray.x = (uv_rect.x - cx() - Tx()) / fx();
    ray.y = (uv_rect.y - cy() - Ty()) / fy();
    ray.z = 1.0;
    return ray;
}

void rectifyImage(const sensor_msgs::Image &msg, cv::Mat &raw, cv::Mat &undistort)
{
    assert(initialized());

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    raw = cv_ptr->image;

    cv::Mat R_ = cv::Mat_<double>::eye(3, 3);
    cv::Matx33d P_binned;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P_binned(i, j) = P_(i, j);
        }
    }

    cv::Size binned_resolution = dst_size_;
    cv::Mat full_map1, full_map2;
    cv::initUndistortRectifyMap(K_, D_, R_, P_binned, binned_resolution,
                                CV_16SC2, full_map1, full_map2);
    cv::remap(raw, undistort, full_map1, full_map2, cv::INTER_LINEAR);

    return;
}

cv::Point2d rectifyPoint(const cv::Point2d &uv_raw)
{
    assert(initialized());

    /// @todo cv::undistortPoints requires the point data to be float, should allow double
    cv::Point2f raw32 = uv_raw, rect32;
    const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
    cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
    return rect32;
}

cv::Point2d unrectifyPoint(const cv::Point2d &uv_rect)
{
    assert(initialized());
    // Convert to a ray
    cv::Point3d ray = projectPixelTo3dRay(uv_rect);

    // Project the ray on the image
    cv::Mat R_ = cv::Mat_<double>::eye(3, 3);
    cv::Mat r_vec, t_vec = cv::Mat_<double>::zeros(3, 1);
    cv::Rodrigues(R_.t(), r_vec);
    std::vector<cv::Point2d> image_point;
    cv::projectPoints(std::vector<cv::Point3d>(1, ray), r_vec, t_vec, K_, D_, image_point);

    return image_point[0];
}

cv::Rect rectifyRoi(const cv::Rect &roi_raw)
{
    assert(initialized());

    /// @todo Actually implement "best fit" as described by REP 104.

    // For now, just unrectify the four corners and take the bounding box.
    cv::Point2d rect_tl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y));
    cv::Point2d rect_tr = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width, roi_raw.y));
    cv::Point2d rect_br = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width,
                                                   roi_raw.y + roi_raw.height));
    cv::Point2d rect_bl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y + roi_raw.height));

    cv::Point roi_tl(std::ceil(std::min(rect_tl.x, rect_bl.x)),
                     std::ceil(std::min(rect_tl.y, rect_tr.y)));
    cv::Point roi_br(std::floor(std::max(rect_tr.x, rect_br.x)),
                     std::floor(std::max(rect_bl.y, rect_br.y)));

    return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

cv::Rect unrectifyRoi(const cv::Rect &roi_rect)
{
    assert(initialized());

    /// @todo Actually implement "best fit" as described by REP 104.

    // For now, just unrectify the four corners and take the bounding box.
    cv::Point2d raw_tl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y));
    cv::Point2d raw_tr = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width, roi_rect.y));
    cv::Point2d raw_br = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width,
                                                    roi_rect.y + roi_rect.height));
    cv::Point2d raw_bl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y + roi_rect.height));

    cv::Point roi_tl(std::floor(std::min(raw_tl.x, raw_bl.x)),
                     std::floor(std::min(raw_tl.y, raw_tr.y)));
    cv::Point roi_br(std::ceil(std::max(raw_tr.x, raw_br.x)),
                     std::ceil(std::max(raw_bl.y, raw_br.y)));

    return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}