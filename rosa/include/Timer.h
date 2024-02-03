
#ifndef ROS_TIMER_H
#define ROS_TIMER_H

#include "rclcpp/rclcpp.hpp"


class Timer
{
public:
    template<typename Object, typename Duration, typename Method, typename... Args>
    void createWallTimer(Object *node, Duration duration, Method&& method, Args&& ... args)
    {
        auto wallTimerFunction = [method, args..., node] () {
            (node->*method)(args...);
        };

        _timer = node->create_wall_timer(duration, wallTimerFunction);
    }

    template<typename Object, typename Duration, typename Method>
    void createWallTimer(Object *node, Duration duration, Method &&method)
    {
        auto wallTimerFunction = [method,  node] () {
            (node->*method)();
        };
        _timer = node->create_wall_timer(duration, wallTimerFunction);
    }

private:
    rclcpp::TimerBase::SharedPtr _timer;
};


#endif //ROS_TIMER_H
