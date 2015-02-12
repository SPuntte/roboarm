/*
Robotic Arm Mouse Control with Inverse Kinematics

Licenced under the MIT License:

Copyright (c) 2015 Pontus Lundström

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

--
This program controls a servo-driven, turntable-shoulder-elbow topology robotic
arm by mouse with a static Inverse Kinematics (IK) solver. It sends servo PWM
commands over serial connection to an ATmega MCU which does the actual servo
driving.

Written hastily in (ugly) C++11 - downgradeable to C++03 with minimal changes.

Dependencies:
  GLFW 3.x
  boost::asio

On UNIX-like systems, compile with:
$ g++ -std=c++11 arm_control.cpp -o arm_control -lglfw -lboost_system

Example of use:
$ ./arm_control /dev/ttyUSB0

*/

#include <boost/asio.hpp>

#include <GLFW/glfw3.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>

#define NDEBUG // Comment/remove to enable debug output

#ifdef NDEBUG
#define DEBUG_OUTPUT(a)
#else
#define DEBUG_OUTPUT(a) a
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{

// Class for mapping angles to servo PWM data
class servo_mapping
{
public:
    servo_mapping(double min_angle, double max_angle,
                  double min_delay, double max_delay) :
        m_min_angle(min_angle),
        m_max_angle(max_angle),
        m_min_delay(min_delay),
        m_max_delay(max_delay)
    {}
    double min_angle() const { return m_min_angle; }
    double max_angle() const { return m_max_angle; }
    double min_delay() const { return m_min_delay; }
    double max_delay() const { return m_max_delay; }
    double delay_from_angle(double angle) const
    {
        // Interpolate linearly & clamp to [m_min_delay, m_max_delay]
        double delay = m_min_delay + (angle - m_min_angle) *
                       (m_max_delay - m_min_delay) / (m_max_angle - m_min_angle);
        if (m_min_delay > m_max_delay)
        {
            return (delay > m_min_delay) ? m_min_delay :
                   ((delay < m_max_delay) ? m_max_delay : delay);    
        }
        return (delay < m_min_delay) ? m_min_delay :
               ((delay > m_max_delay) ? m_max_delay : delay);
    }
    double angle_from_delay(double delay) const
    {
        // Interpolate linearly & clamp to [m_min_angle, m_max_angle]
        double angle = m_min_angle + (delay - m_min_delay) *
                       (m_max_angle - m_min_angle) / (m_max_delay - m_min_delay);
        if (m_min_angle > m_max_angle)
        {
            return (angle > m_min_angle) ? m_min_angle :
                   ((angle < m_max_angle) ? m_max_angle : angle);    
        }
        return (angle < m_min_angle) ? m_min_angle :
               ((angle > m_max_angle) ? m_max_angle : angle);
    }
private:
    double m_min_angle;
    double m_max_angle;
    double m_min_delay;
    double m_max_delay;
};

// Data structures for IK
struct joint
{
    double d;
    double s;
    double theta;
    double theta_min;
    double theta_max;
};

struct point
{
    double x;
    double y;
    double z;
};

std::array<joint*, 3> arm_joints{nullptr, nullptr, nullptr};
point* wrist_position = nullptr;

// Static IK scheme for turntable-shoulder-elbow topology
// Returns the number of valid solutions (0...2)
int ik(point const& goal, joint& j1, joint& j2, joint& j3)
{
    double epsilon = 1e-10;

    const auto& px = goal.x;
    const auto& py = goal.y;
    const auto& pz = goal.z;

    const auto& s1 = j1.s;
    
    // Solve for turntable
    double D = sqrt(py*py*(px*px + py*py - s1*s1));
    j1.d = D / py;
    j1.theta = atan2((py*py*s1 - px*D) / (px*px*py + py*py*py),
                     (px*s1 + D) / (px*px + py*py));

    const auto& d1 = j1.d;
    const auto& d2 = j2.d;
    const auto& d3 = j3.d;

    const auto& s2 = j2.s;
    const auto& s3 = j3.s;

    double d12 = d1*d1;
    double d13 = d1*d12;
    double d14 = d12*d12;
    double d22 = d2*d2;
    double d23 = d2*d22;
    double d24 = d22*d22;
    double d32 = d3*d3;
    double d33 = d3*d32;
    double pz2 = pz*pz;
    double pz3 = pz*pz2;
    double s22 = s2*s2;
    double s23 = s2*s22;
    double s32 = s3*s3;
    double s33 = s3*s32;

    double temp1 = d2*pz - d1*s2;
    double temp2 = d12 + pz2;
    double radicand = -temp1*temp1*(d14 - 2*(d22 + d32 - pz2 + s22 + s32)*d12 + d24 + d32*d32 + pz2*pz2 + s22*s22 + s32*s32 - 2*d32*pz2 - 2*d32*s22 - 2*pz2*s22 + 2*d32*s32 - 2*pz2*s32 - 2*s22*s32 - 2*d22*(d32 + pz2 - s22 + s32));

    // Neither division by zero nor negative radicand of square root is allowed
    if (abs(temp1) < epsilon || temp2 < epsilon || radicand < 0)
    {
        if (abs(temp1) < epsilon)
        {
            DEBUG_OUTPUT(std::cerr << "abs(temp1) < epsilon" << std::endl);
        }
        if (temp2 < epsilon)
        {
            DEBUG_OUTPUT(std::cerr << "temp2 < epsilon" << std::endl);
        }
        if (radicand < 0)
        {
            DEBUG_OUTPUT(std::cerr << "radicand < 0" << std::endl);
        }
        // NO SOLUTION!
        return 0;
    }
    else
    {
        double temp3 = sqrt(radicand);

        // Solution 1
        double theta2_1 = atan2(
        -(s22*d14 - 2*d2*pz*s2*d13 + ((pz2 + s22)*d22 + s22*(-d32 + pz2 + s22 - s32))*d12 + (-2*s2*pz3 - 2*s23*pz + 2*s2*s32*pz - 2*d22*s2*pz + 2*d32*s2*pz + temp3)*d2*d1 + (pz*d24 + pz*(-d32 + pz2 + s22 - s32)*d22 + temp3*s2)*pz)/(-temp1*temp2),
        (d2*d13 + pz*s2*d12 + d2*(d22 - d32 + pz2 + s22 - s32)*d1 + pz*s23 - pz*s2*s32 - temp3 + pz3*s2 + d22*pz*s2 - d32*pz*s2)/temp2);

        double theta3_1 = atan2(
        (pz*s3*d24 + pz*s33*d22 - pz3*s3*d22 + pz*s22*s3*d22 + d32*pz*s3*d22 - d3*pz*s23*d2 - d3*pz*s2*s32*d2 - d3*temp3*d2 + d3*pz3*s2*d2 - d33*pz*s2*d2 + d12*pz*(d3*s2 - d2*s3)*d2 - s2*s3*temp3 - d23*d3*pz*s2 + d13*s2*(d2*s3 - d3*s2) + d1*s2*(d3*s2 - d2*s3)*(d22 + d32 - pz2 + s22 + s32))/temp1,
        -(d3*pz*d24 + pz*s2*s3*d23 + d3*pz*(d32 - pz2 + s22 + s32)*d22 + (-s2*pz3 + s23*pz + s2*s32*pz + d32*s2*pz + temp3)*s3*d2 - d12*pz*(d2*d3 + s2*s3)*d2 - d3*s2*temp3 + d13*s2*(d2*d3 + s2*s3) - d1*s2*(d2*d3 + s2*s3)*(d22 + d32 - pz2 + s22 + s32))/temp1);

        // Solution 2
        double theta2_2 = atan2(
        (-s22*d14 + 2*d2*pz*s2*d13-((pz2 + s22)*d22 + s22*(-d32 + pz2 + s22 - s32))*d12 + (2*s2*pz3 + 2*s23*pz - 2*s2*s32*pz + 2*d22*s2*pz - 2*d32*s2*pz + temp3)*d2*d1 + (pz*(d32 - pz2 - s22 + s32)*d22 - d24*pz + temp3*s2)*pz)/(-temp1*temp2),
        (d2*d13 + pz*s2*d12 + d2*(d22 - d32 + pz2 + s22 - s32)*d1 + pz*s23 - pz*s2*s32 + temp3 + pz3*s2 + d22*pz*s2 - d32*pz*s2)/temp2);

        double theta3_2 = atan2(
        (pz*s3*d24 + pz*s33*d22 - pz3*s3*d22 + pz*s22*s3*d22 + d32*pz*s3*d22 - d3*pz*s23*d2 - d3*pz*s2*s32*d2 + temp3*d3*d2 + d3*pz3*s2*d2 - d33*pz*s2*d2 + d12*pz*(d3*s2 - d2*s3)*d2 - d23*d3*pz*s2 + temp3*s2*s3 + d13*s2*(d2*s3 - d3*s2) + d1*s2*(d3*s2 - d2*s3)*(d22 + d32 - pz2 + s22 + s32))/temp1,
        -(d3*pz*d24 + pz*s2*s3*d23 - d3*pz3*d22 + d3*pz*s22*d22 + d3*pz*s32*d22 + d33*pz*d22 + pz*s2*s33*d2 - s3*temp3*d2 + pz*s23*s3*d2 - pz3*s2*s3*d2 + d32*pz*s2*s3*d2 - d12*pz*(d2*d3 + s2*s3)*d2 + temp3*d3*s2 + d13*s2*(d2*d3 + s2*s3) - d1*s2*(d2*d3 + s2*s3)*(d22 + d32 - pz2 + s22 + s32))/temp1);

        // Choose which solution is better
        bool solution1_inbound = true;
        if (theta2_1 < j2.theta_min || theta2_1 > j2.theta_max || theta3_1 < j3.theta_min || theta3_1 > j3.theta_max)
        {
            // Solution 1 is out of bounds
            solution1_inbound = false;
        }
        if (theta2_2 < j2.theta_min || theta2_2 > j2.theta_max || theta3_2 < j3.theta_min || theta3_2 > j3.theta_max)
        {
            // Solution 2 is out of bounds
            DEBUG_OUTPUT(std::cerr << "Solution 2 is out of bounds." << std::endl);
            if (solution1_inbound)
            {
                // Solution 1 is the only valid solution
                j2.theta = theta2_1;
                j3.theta = theta3_1;
                return 1;
            }
            else
            {
                DEBUG_OUTPUT(std::cerr << "Both solutions are out of bounds." << std::endl);
                // NO VALID SOLUTION!
                return 0;
            }
        }    
        else
        {
            if (solution1_inbound)
            {
                // Both solutions are valid -> choose one for which theta2 is greater
                if (theta2_1 > theta2_2)
                {
                    j2.theta = theta2_1;
                    j3.theta = theta3_1;
                    return 2;
                }
                else
                {
                    j2.theta = theta2_2;
                    j3.theta = theta3_2;
                    return 2;
                }
            }
            else
            {
                DEBUG_OUTPUT(std::cerr << "Solution 1 is out of bounds." << std::endl);
                // Solution 2 is the only valid solution
                j2.theta = theta2_2;
                j3.theta = theta3_2;
                return 1;
            }
        }
    }
}

// Invokes IK for input point and updates position if solution exists
void move_arm(point const& new_position)
{
    joint new_joints[3];
    for (size_t i = 0; i < 3; ++i)
    {
        new_joints[i] = *arm_joints[i];
    }

    if (ik(new_position, new_joints[0], new_joints[1], new_joints[2]) > 0)
    {
        *wrist_position = new_position;
        for (size_t i = 0; i < 3; ++i)
        {
            *arm_joints[i] = new_joints[i];
        }        
    }
}

// Wrist planar position (xy-plane) control callback
void cursor_callback(GLFWwindow*, double cursor_x, double cursor_y)
{
    const double cursor_sensitivity = 0.02;

    static double old_x = 0.0;
    static double old_y = 0.0;

    double delta_x = cursor_x - old_x;
    double delta_y = cursor_y - old_y;

    point new_position{wrist_position->x + cursor_sensitivity * delta_x,
                       wrist_position->y - cursor_sensitivity * delta_y,
                       wrist_position->z};

    // Ignore initial mouse position event
    static bool debounce = true;
    if (!debounce)
    {    
        move_arm(new_position);
    }
    else
    {
        debounce = false;
    }

    old_x = cursor_x;
    old_y = cursor_y;
}

// Wrist height (z-axis) control callback
void scroll_callback(GLFWwindow*, double, double offset)
{
    const double scroll_sensitivity = 3.0;
    
    point new_position{wrist_position->x, wrist_position->y,
                       wrist_position->z + scroll_sensitivity * offset};
    
    // Ignore initial mouse position event
    static bool debounce = true;
    if (!debounce)
    {
        move_arm(new_position); 
    }
    else
    {
        debounce = false;
    }
}

template <typename T>
T deg2rad(T a)
{
    return a*T(M_PI/180);
}

template <typename T>
T rad2deg(T a)
{
    return a*T(180/M_PI);
}

}

int main(int argc, char const *argv[])
{
    try
    {
        // Set fixed float width for stdout
        std::cout.setf(std::ios::fixed, std:: ios::floatfield);
        std::cout.precision(1);

        if (argc < 2)
        {
            std::cout << "Usage:\narm_control <serial_device>" << std::endl;
            return 0;
        }

        auto serial_device = argv[1];
        std::cout << "Begin robot arm control on serial interface '" << serial_device << "'\n\n"
                  << "Can't find the mouse cursor? "
                  << "Don't panic, this is necessary to capture mouse input.\n"
                  << "Press Q or Alt+F4 to kill the program and regain mouse control.\n" << std::endl;

        //=== Arm configuration parameters =========================================

        // Joint angle limits
        const double turntable_min_angle = deg2rad( -90.0);
        const double turntable_max_angle = deg2rad(  90.0);
        const double shoulder_min_angle  = deg2rad( -11.5);
        const double shoulder_max_angle  = deg2rad( 136.0);
        // TODO: measure
        const double elbow_min_angle     = deg2rad(-170.0);
        const double elbow_max_angle     = deg2rad(  10.0);

        std::array<servo_mapping, 3> servos =
            {servo_mapping{turntable_min_angle, turntable_max_angle,  600, 2315},
             servo_mapping{shoulder_min_angle,  shoulder_max_angle,  2100,  650},
             servo_mapping{elbow_min_angle,     elbow_max_angle,     2100,  700}};

        // Joint topology, lengths in millimeters (mm)
        joint j1{0.0,   -23.0, 0.0, turntable_min_angle, turntable_max_angle};
        joint j2{248.0, -25.0, 0.0,  shoulder_min_angle,  shoulder_max_angle};
        joint j3{272.0,   0.0, 0.0,     elbow_min_angle,     elbow_max_angle};
        arm_joints[0] = &j1;
        arm_joints[1] = &j2;
        arm_joints[2] = &j3;

        // Initial wrist position
        point goal{0.0, 100.0, 50.0};
        wrist_position = &goal;
        //=== Arm configuration parameters =========================================

        // Initialize joint angles with IK
        if (ik(goal, j1, j2, j3) == 0)
        {
            std::cerr << "Initial position is not reachable!" << std::endl;
            return -1337;
        }

        // Initialize serial communication
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, serial_device);
        serial.set_option(boost::asio::serial_port::baud_rate(57600));

        // Initialize GLFW
        if (!glfwInit())
            return -1;

        // Create a windowed mode window and its OpenGL context */
        auto window = glfwCreateWindow(320, 240, "Robot arm control", NULL, NULL);
        if (!window)
        {
            glfwTerminate();
            return -1;
        }

        // Make the window's rendering context current
        glfwMakeContextCurrent(window);

        // This hides the cursor and enables infinite cursor movement
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Setup mouse event callbacks
        glfwSetScrollCallback(window, scroll_callback);
        glfwSetCursorPosCallback(window, cursor_callback);

        // Loop until the user closes the window */
        while (!glfwWindowShouldClose(window))
        {
            glfwSwapBuffers(window);

            // Poll and process events
            glfwPollEvents();

            std::cout << "\rWrist position: [" << goal.x << ", " << goal.y << ", " << goal.z << "]"
                      << "  Joint angles: [" << rad2deg(arm_joints[0]->theta) << ", "
                                             << rad2deg(arm_joints[1]->theta) << ", "
                                             << rad2deg(arm_joints[2]->theta) << "] ";
            for (size_t i = 0; i < 3; ++i)
            {
                DEBUG_OUTPUT(std::cerr << "Send command for servo #" << i << std::endl);
                // Get delay from angle & encode servo index
                uint16_t command = servos[i].delay_from_angle(arm_joints[i]->theta);

                DEBUG_OUTPUT(std::cerr << "\t" << command << std::endl);
                command |= (i << 12);

                // Send servo command
                boost::asio::write(serial, boost::asio::buffer(&command, 2));
            }
            std::cout << "          " << std::flush;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
                break;
        }
        std::cout << "\n" << std::endl;
        glfwTerminate();
        
        return 0;
    }
    catch (boost::system::system_error& e)
    {
        std::cerr << "\n\nSerial communication error: " << e.what() << "\n" << std::endl;
        glfwTerminate();
        return -1;
    }
    catch (std::exception& e)
    {
        std::cerr << "\n\nSomething went wrong: " << e.what() << "\n" << std::endl;
        glfwTerminate();
        return -2;
    }
    catch (...)
    {
        std::cerr << "\n\nUnknown error! :(" << "\n" << std::endl;
        glfwTerminate();
        return -666;
    }
}
