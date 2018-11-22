
#include <cmath>

class Angle {
public:
    inline Angle() {
        angle = 0;
    }

    inline Angle(float angle) {
        if (angle >= 360) {
            this->angle = angle - 360 * floor(angle / 360);
        } else if (angle < 0) {
            this->angle = angle + 360 * ceil(-angle / 360);
        } else {
            this->angle = angle;
        }
    }

    inline float as_float() {
        return angle;
    }

    friend Angle operator+(const Angle &a, const float b) {
        return Angle(a.angle + b);
    }

    friend Angle operator-(const Angle &a, const float b) {
        return Angle(a.angle - b);
    }

    inline bool operator==(const Angle &a) const {
        return this->angle == a.angle;
    }

    inline float distance(const Angle &other) {
        float distance = other.angle - this->angle;
        if (distance < 0) distance += 360;
        if (distance > 180) distance = -(360 - distance);
        return distance;
    }

private:
    float angle;
};
