/**
 * @file    KeyframePublisher.hpp
 * @brief   Publish custom Keyframe message
 * @author  Jianzhu Huai
 */

#ifndef KEYFRAME_PUBLISHER_HPP
#define KEYFRAME_PUBLISHER_HPP

#include <ros/ros.h>
#include "loop_closure/KeyframeForLoopDetection.hpp"

namespace swift_vio {

class KeyframePublisher {
public:
KeyframePublisher(ros::NodeHandle& n);

/**
 * wait for at least one subscriber if we don't want to lose the first few messages.
 */
size_t waitForSubscriber(bool performLoopClosure);

void publish(const LoopQueryKeyframeMessage& message) const;

private:
ros::Publisher publisher_;
mutable size_t numPublishedKeyframes_;
const size_t maxLoopFrameQueueSize_ = 20;

}; // KeyframePublisher

} // namespace swift_vio

#endif // KEYFRAME_PUBLISHER_HPP

