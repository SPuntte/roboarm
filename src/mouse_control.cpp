#include <GLFW/glfw3.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#define NDEBUG

#ifdef NDEBUG
#define IK_DEBUG_OUTPUT(a)
#else
#define IK_DEBUG_OUTPUT(a) a
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{

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

joint* arm_joints[3] = {nullptr, nullptr, nullptr};
point* wrist_position = nullptr;

// Solve Inverse Kinematics, return number of solutions found
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
            IK_DEBUG_OUTPUT(std::cerr << "abs(temp1) < epsilon" << std::endl);
        }
        if (temp2 < epsilon)
        {
            IK_DEBUG_OUTPUT(std::cerr << "temp2 < epsilon" << std::endl);
        }
        if (radicand < 0)
        {
            IK_DEBUG_OUTPUT(std::cerr << "radicand < 0" << std::endl);
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
            IK_DEBUG_OUTPUT(std::cerr << "Solution 2 is out of bounds." << std::endl);
            if (solution1_inbound)
            {
                // Solution 1 is the only valid solution
                j2.theta = theta2_1;
                j3.theta = theta3_1;
                return 1;
            }
            else
            {
                IK_DEBUG_OUTPUT(std::cerr << "Both solutions are out of bounds." << std::endl);
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
                IK_DEBUG_OUTPUT(std::cerr << "Solution 1 is out of bounds." << std::endl);
                // Solution 2 is the only valid solution
                j2.theta = theta2_2;
                j3.theta = theta3_2;
                return 1;
            }
        }
    }
}

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

void cursor_callback(GLFWwindow*, double cursor_x, double cursor_y)
{
    const double cursor_sensitivity = 0.05;
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

void scroll_callback(GLFWwindow*, double, double offset)
{
    const double scroll_sensitivity = 1.0;
    
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

int main()
{
    // Arm configuration, lengths in millimeters (mm)
    joint j1{0.0, 0.0, 0.0, deg2rad(-90.0), deg2rad(90.0)};
    joint j2{250.0, 0.0, 0.0, deg2rad(-11.5), deg2rad(136.0)};
    joint j3{300.0, 0.0, 0.0, deg2rad(-170.0), deg2rad(10.0)};
    arm_joints[0] = &j1;
    arm_joints[1] = &j2;
    arm_joints[2] = &j3;

    point goal{0.0, 150.0, 50.0};
    wrist_position = &goal;

    // Set fixed float width for stdout
    std::cout.setf(std::ios::fixed, std:: ios::floatfield);
    std::cout.precision(1);

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

    /* Make the window's context current */
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

        // Poll for and process events
        glfwPollEvents();

        std::cout << "\rWrist position: [" << goal.x << ", " << goal.y << ", " << goal.z << "]"
                  << "  Joint angles: [" << rad2deg(j1.theta) << ", "
                                         << rad2deg(j2.theta) << ", "
                                         << rad2deg(j3.theta) << "]          " << std::flush;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    glfwTerminate();

    std::cout << std::endl;
    return 0;
}