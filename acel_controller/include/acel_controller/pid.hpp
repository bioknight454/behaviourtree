#include <math.h>
class PID
{
private:
    struct heading_param
    {
        float kp;
        float ki;
        float kd;
    } headingParams;

    struct base_param
    {
        float kp;
        float ki;
        float kd;
    } baseParams;

    struct error
    {
        float proportional;
        float integral;
        float derivative;
        float previous;
    } err;

    struct velocity
    {
        float linear;
        float angular;
        float angular_filter;
    } vel;
    struct out
    {
        float heading;
        float linear;
    } output;
    float u;

public:
    void setBaseParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    void setHeadingParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    float control_base_(float error, float speed)
    {

        err.proportional = error;

        err.integral += err.proportional;

        err.derivative = (err.proportional - err.previous);

        err.previous = err.proportional;

        u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;

        return fmax(-speed, fmin(u, speed));
    }

    float control_base_(float error, float speed, float deltaT)
    {

        err.proportional = error;

        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous)/ deltaT;

        err.previous = err.proportional;

        u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;

        return fmax(-speed, fmin(u, speed));
    }

    float control_base(float error, float speed, int condition, float deltaT)
    {
        if (condition)
        {
            err.proportional = error;
            if (err.proportional > 180)
            {
                err.proportional -= 360;
            }
            else if (err.proportional < -180)
            {
                err.proportional += 360;
            }
        }
        else
        {
            err.proportional = error;
        }
        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous) / deltaT;

        err.previous = err.proportional;

        output.linear = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;
        output.heading = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * err.derivative;
        return condition ? fmax(-speed, fmin(output.heading, speed)) : fmax(-speed, fmin(output.linear, speed));
    }
    float control_base(float error, float speed, int condition)
    {
        if (condition)
        {
            err.proportional = error;
            if (err.proportional > 180)
            {
                err.proportional -= 360;
            }
            else if (err.proportional < -180)
            {
                err.proportional += 360;
            }
        }
        else
        {
            err.proportional = error;
        }
        err.integral += err.proportional;

        err.derivative = (err.proportional - err.previous);

        err.previous = err.proportional;

        float u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * err.derivative;
        float uT = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * err.derivative;
        return condition ? fmax(-speed, fmin(uT, speed)) : fmax(-speed, fmin(u, speed));
    }
    float getErrorP() const
    {
        return err.proportional;
    }
    float getErrorI() const
    {
        return err.integral;
    }
    float getErrorD() const
    {
        return err.derivative;
    }
    float getHeadOut() const
    {
        return output.heading;
    }
    float getLinearOut() const
    {
        return output.linear;
    }
    float getU() const
    {
        return u;
    }
};
