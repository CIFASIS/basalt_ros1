/* -*-c++-*--------------------------------------------------------------------
 * 2020 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <basalt_ros1/image_subscriber.h>

#include <basalt/optical_flow/optical_flow.h>

#include <boost/range/irange.hpp>

#include <cv_bridge/cv_bridge.h>

#include <stdexcept>

#include <chrono>

using boost::irange;

namespace basalt_ros1 {
ImageSubscriber::ImageSubscriber(const ros::NodeHandle& node,
                                 const std::string& topic_left,
                                 const std::string& topic_right)
    : node_(node) {
  subs_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
      node_, topic_left, 2));
  subs_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
      node_, topic_right, 2));
  sync_ = std::make_shared<message_filters::TimeSynchronizer<Image, Image>>(
      *subs_[0], *subs_[1], 10);
  sync_->registerCallback(&ImageSubscriber::callback, this);
  lastTime_ = ros::Time::now();
//#ifdef SAVE_TIMES
//  f_track_times_.open("tracking_times_start.txt");
//  f_track_times_ << std::fixed;
//  f_track_times_ << std::setprecision(6);
//  num_tracked_frames_ = 0;
//#endif
}

ImageSubscriber::~ImageSubscriber() {
//#ifdef SAVE_TIMES
//  f_track_times_.close();
//#endif
}


void ImageSubscriber::callback(ImageConstPtr const& left,
                               ImageConstPtr const& right) {
//#ifdef SAVE_TIMES
//  num_tracked_frames_++;
//  auto const t = std::chrono::system_clock::now().time_since_epoch().count();
//  f_track_times_ << num_tracked_frames_ << " " << t / 1e09 << std::endl;
//#endif
  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

  const int NUM_CAMS = 2;
  data->img_data.resize(NUM_CAMS);
  data->t_ns = left->header.stamp.sec * 1000000000LL + left->header.stamp.nsec;
  const ImageConstPtr msg[2] = {left, right};
  for (const auto& i : irange(0, NUM_CAMS)) {
    const auto& img = *msg[i];
    data->img_data[i].exposure =
        0;  /// TODO: use data RS2_FRAME_METADATA_ACTUAL_EXPOSURE * 1e-6

    data->img_data[i].img.reset(
        new basalt::ManagedImage<uint16_t>(img.width, img.height));
    std::string encoding = img.encoding;
    if(encoding == "mono8"){
        const uint8_t* data_in = (const uint8_t*)&img.data[0];
        uint16_t* data_out = data->img_data[i].img->ptr;
        // TODO: this 8-bit to 16bit conversion can probably be done
        // more efficiently with opencv
        size_t full_size = img.width * img.height;
        for (size_t j = 0; j < full_size; j++) {
          int val = data_in[j];
          val = val << 8;
          data_out[j] = val;
        }
    } else if (encoding == "bgr8") {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
        // Convert cv::Mat (Unsigned 2 byte integer) to Pangolin ManagedImage
        std::memcpy((void*)data->img_data[i].img->ptr, (void*)cv_ptr->image.data, cv_ptr->image.total()*cv_ptr->image.elemSize());
    } else {
        std::cerr << "Encoding " << encoding << " is not supported."
                 << std::endl;
        std::abort();
    }
  }

  framesReceived_++;
  if (framesReceived_ == 100) {
    const auto t = ros::Time::now();
    const auto dt = t - lastTime_;
    ROS_INFO_STREAM("received image frame rate: " << framesReceived_ /
                                                         dt.toSec());
    framesReceived_ = 0;
    lastTime_ = t;
  }

  if (queue_) {
    if (!queue_->try_push(data)) {
      ROS_WARN_STREAM("image frame " << data->t_ns
                                     << " dropped due to queue overflow");
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros1
