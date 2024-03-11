
#ifndef ROS_SERVICE_H
#define ROS_SERVICE_H

#include "rclcpp/rclcpp.hpp"

#include <utility>


template<typename ServiceT>
class Service
{
public:
    Service(rclcpp::Node* parent, std::string serviceName);

protected:
    virtual void processRequest(std::shared_ptr<typename ServiceT::Request> request,
                        std::shared_ptr<typename ServiceT::Response> response) = 0;

    void createService(std::string const& serviceName);

    rclcpp::Logger getLogger();

private:
    typename rclcpp::Service<ServiceT>::SharedPtr _service;
    rclcpp::Node* _parent;
};


template<typename ServiceT>
Service<ServiceT>::Service(rclcpp::Node* parent, std::string serviceName)
    : _parent(parent)
{
    createService(serviceName);
}

template<typename ServiceT>
void Service<ServiceT>::createService(std::string const& serviceName)
{
    auto processRequestPointer =
        [this](std::shared_ptr<typename ServiceT::Request> request,
            std::shared_ptr<typename ServiceT::Response> response) {
            this->processRequest(request, response);
        };
    _service = _parent->create_service<ServiceT>(serviceName, processRequestPointer);

    RCLCPP_INFO(_parent->get_logger(), "%s service started", serviceName.c_str());
}

template<typename ServiceT>
rclcpp::Logger Service<ServiceT>::getLogger()
{
    return _parent->get_logger();
}

#endif //ROS_SERVICE_H
