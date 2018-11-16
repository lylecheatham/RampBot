/* This code courtesy of Kris Winer, adapted for MTE 380 Robot Project:
 *
 * by: Kris Winer
 * date: April 1, 2014
 * license: Beerware - Use this code however you'd like. If you
 * find it useful you can buy me a beer some time.
 *
 */


#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#include <Arduino.h>
#include <array>

class QuaternionFilter{
public:
    QuaternionFilter();

    void madgwickUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    void mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    const std::array<float, 4>& getQ();

private:
    // Vector to hold integral error for Mahony method
    std::array<float, 3> eInt;
    // Vector to hold quaternion
    std::array<float, 4> q;
};

#endif  // _QUATERNIONFILTERS_H_
