//
// Implementation for the Pose2 class
//
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include "types/Pose2.hpp"

Pose2::Pose2() : _x(0), _y(0), _yaw(0), _velocity(0) {}

Pose2::Pose2(double x, double y, double yaw) : _x(x), _y(y), _yaw(wrapAngle(yaw)), _velocity(0) {}

Pose2::Pose2(Eigen::Matrix4f m) {
	_x = m(0,3);
	_y = m(1,3);
	_yaw = atan2(m(1,0),m(0,0));
	_velocity = 0;
}

Pose2::Pose2(geometry_msgs::Pose p, double yaw_offset) : Pose2(p.position.x, p.position.y,
                                                                            tf::getYaw(p.orientation) + yaw_offset) {}

Pose2 Pose2::operator+(const Pose2& innovation) {
    double result_x   = innovation.x() * cos(yaw()) - innovation.y() * sin(yaw()) + x();
    double result_y   = innovation.x() * sin(yaw()) + innovation.y() * cos(yaw()) + y();
    double result_yaw = innovation.yaw() + yaw();

    return Pose2(result_x, result_y, result_yaw);
}

Pose2 Pose2::operator-(const Pose2& innovation) const {
    double theta = innovation.yaw();
    double diff_x = x() - innovation.x();
    double diff_y = y() - innovation.y();
    double result_x   = diff_x * cos(theta) + diff_y * sin(theta);
    double result_y   = -diff_x * sin(theta) + diff_y * cos(theta);
    double result_yaw = yaw() - theta;
    return Pose2(result_x, result_y, result_yaw);
}

Pose2 Pose2::operator* (const double scale) {
    double result_yaw = yaw();
    double result_x = scale * x();
    double result_y = scale * y();

    return Pose2(result_x, result_y, result_yaw);
}

Pose2 Pose2::operator=(const Pose2& other) const {
    Pose2 res = Pose2(other.x(), other.y(), other.yaw());
    res.setVelocity(other.velocity());
    return res;
}

bool Pose2::operator==(const Pose2 other) {
    return fabs(x() - other.x()) < 1e-10 && fabs(y() - other.y()) < 1e-10 && fabs(yaw() - other.yaw()) < 1e-10;
}

void Pose2::operator=(const Pose2 &position) {
    _x = position.x();
    _y = position.y();
    _yaw = position.yaw();
    _velocity = position.velocity();
}

std::ostream& operator<<(std::ostream &os, const Pose2 &pos) {
    os << "x: " << pos.x() << " y: " << pos.y() << " yaw: " << pos.yaw();
    return os;
}

void Pose2::print(std::string str) const {
    std::cout << str << std::fixed << " x: " << x() << "    " << "y: " << y() << "    " <<
                                                                                        "yaw: " << yaw() << std::endl;
}

const double& Pose2::x() const {
    return _x;
}
const double& Pose2::y() const {
    return _y;
}
const double& Pose2::yaw() const {
    return _yaw;
}
const double& Pose2::velocity() const {
    return _velocity;
}

void Pose2::setX(double x) {
    _x = x;
}

void Pose2::setY(double y) {
    _y = y;
}

void Pose2::setYaw(double yaw) {
    _yaw = wrapAngle(yaw);
}

void Pose2::setVelocity(double vel) {
    _velocity = vel;
}

void Pose2::rotate(double ang_rad) {
    // Rotate counterclockwise
    setX(x() * cos(ang_rad) - y() * sin(ang_rad));
    setY(x() * sin(ang_rad) + y() * cos(ang_rad));
    setYaw(yaw() + ang_rad);
}

Pose2 Pose2::difference(Pose2 innovation) const {
    double diff_x = x() - innovation.x();
    double diff_y = y() - innovation.y();
    double diff_yaw = yaw() - innovation.yaw();
    return Pose2(diff_x, diff_y, diff_yaw);
}

double Pose2::norm() const {
    return sqrt(x() * x() + y() * y());
}

double Pose2::distance(const Pose2 &other) const {
    double diff_x = other.x() - x();
    double diff_y = other.y() - y();
    return sqrt((diff_x * diff_x) + (diff_y * diff_y));
}
double Pose2::squared_norm() const {
    return _x * _x + _y * _y;
}

double Pose2::squared_distance(const Pose2 &other) const {
    double diff_x = other.x() - x();
    double diff_y = other.y() - y();
    return (diff_x * diff_x) + (diff_y * diff_y);
}

double Pose2::dot(const Pose2 &other) const {
    return (x() * other.x() + y() * other.y());
}

double Pose2::wrapAngle(double angle) {
    if (angle <= (double)-M_PI) return wrapAngle(angle + 2.0 * (double)M_PI);
    if (angle > (double)M_PI) return wrapAngle(angle - 2.0 * (double)M_PI);
    return angle;
}

geometry_msgs::Pose Pose2::toMsg() const {
    geometry_msgs::Pose p;
    p.position.x = x();
    p.position.y = y();
    p.position.z = 0;
    p.orientation = tf::createQuaternionMsgFromYaw(yaw());
    return p;
}

geometry_msgs::Pose Pose2::toMsgWithVelocity() const {
    geometry_msgs::Pose p = toMsg();
    p.position.z = _velocity;
    return p;
}
