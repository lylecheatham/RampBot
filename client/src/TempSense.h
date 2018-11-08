#include <cstdint>

class TempSense{
public:
    TempSense(int pin, bool continuous = true);
    ~TempSense();

    void set_B_value(float new_B);
    void set_R_25C_value(float new_R_25C);
    void set_R_bal_value(float new_R_bal);

    float get_temp();
    int32_t get_raw();
    bool is_continuous();

    enum configuration {
        high_side,
        low_side
    };

    static constexpr float default_B = 4267;
    static constexpr float default_R_25C = 100000;
    static constexpr float default_R_bal = 15000;
    static constexpr configuration default_config = high_side;


private:
    float B;
    float R_25C;
    float R_bal;
    float r_inf;

    bool continuous;

    int pin;

    configuration config;
};
