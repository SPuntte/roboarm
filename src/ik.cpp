#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
            std::cout << "abs(temp1) < epsilon" << std::endl;
        }
        if (temp2 < epsilon)
        {
            std::cout << "temp2 < epsilon" << std::endl;
        }
        if (radicand < 0)
        {
            std::cout << "radicand < 0" << std::endl;
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
            std::cout << "Solution 2 is out of bounds." << std::endl;
            if (solution1_inbound)
            {
                // Solution 1 is the only valid solution
                j2.theta = theta2_1;
                j3.theta = theta3_1;
                return 1;
            }
            else
            {
                std::cout << "Both solutions are out of bounds." << std::endl;
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
                std::cout << "Solution 1 is out of bounds." << std::endl;
                // Solution 2 is the only valid solution
                j2.theta = theta2_2;
                j3.theta = theta3_2;
                return 1;
            }
        }
    }
}

int main()
{
    joint j1{0.0, 0.0, 0.0, -M_PI/2.0, M_PI/2.0};
    joint j2{20.0, 0.0, 0.0, -M_PI/2.0, M_PI/2.0};
    joint j3{20.0, 0.0, 0.0, -M_PI/2.0, M_PI/2.0};
    
    point goal{0.0, 39.9, 0.1};

    int num_solutions = ik(goal, j1, j2, j3);
    if (num_solutions == 0)
    {
        std::cout << "No solutions." << std::endl;
    }
    else
    {
        std::cout << "Number of solutions found: " << num_solutions << std::endl;
        std::cout << "Best solution:" << std::endl;
        std::cout << "d1 = " << j1.d << std::endl;
        std::cout << "theta1 = " << j1.theta * 180 / M_PI << std::endl;
        std::cout << "theta2 = " << j2.theta * 180 / M_PI << std::endl;
        std::cout << "theta3 = " << j3.theta * 180 / M_PI << std::endl;
    }
    return 0;
}
