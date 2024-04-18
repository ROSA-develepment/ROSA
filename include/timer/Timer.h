
#ifndef __TIMER_H__
#define __TIMER_H__

#include "rclcpp/rclcpp.hpp"


class Timer
{
public:
    //template<class Object, class Duration, class Callback, class... Args>
    //void createWallTimer(Object *node, Duration duration, Callback&& method, Args&& ... args);
    //
    //template<class Object, class Duration, class Callback>
    //void createWallTimer(Object *node, Duration duration, Callback &&method);

    template<class Object, class Duration>
    void createWallTimer(Object *node, Duration duration, std::function<void()> &&method);

    //template<class Object, class Duration, class... Args>
    //void createWallTimer(Object *node, Duration duration, std::function<void(Args&& ... args)> &&method);

private:
    void cancel();
    rclcpp::TimerBase::SharedPtr _timer;
};


//template<class Object, class Duration, class Callback, class... Args>
//void Timer::createWallTimer(Object *node, Duration duration, Callback&& method, Args&& ... args)
//{
//    auto wallTimerFunction = [method, args..., node] () {
//        (node->*method)(args...);
//    };
//
//    _timer = node->create_wall_timer(duration, wallTimerFunction);
//}
//
//template<class Object, class Duration, class Callback>
//void Timer::createWallTimer(Object *node, Duration duration, Callback &&method)
//{
//    auto wallTimerFunction = [method,  node] () {
//        (node->*method)();
//    };
//    _timer = node->create_wall_timer(duration, wallTimerFunction);
//}

template<class Object, class Duration>
void Timer::createWallTimer(Object *node, Duration duration, std::function<void()> &&method)
{
    _timer = node->create_wall_timer(duration, method);
}

//template<class Object, class Duration, class... Args>
//void Timer::createWallTimer(Object *node, Duration duration, std::function<void(Args &&...)> &&method)
//{
//    _timer = node->create_wall_timer(duration, method);
//}

void Timer::cancel()
{
    _timer->cancel();
}

#endif //__TIMER_H__
