#ifndef __COMP417_PID_H__
#define __COMP417_PID_H__

#include <dynamic_reconfigure/server.h>
#include <wall_following_assignment/PID_parasConfig.h>

class PID {
 public:
  PID(double Kp_init, double Td_init, double Ti_init, double dt_init) {
    // todo: write this
    Kp = Kp_init;
    Td = Td_init;
    Ti = Ti_init;
    dt = dt_init;
    current_error = 0;
    previous_error = 0;
    sum_error = 0;
    current_deriv_error = 0;
    previous_deriv_error = 0;
    control = 0;
  };
  
  void update_control(double current_error) {
    
    // todo: write this

    // calculate the time derivative for error
    current_error = current_error;
    sum_error += current_error;
    current_deriv_error = (current_error - previous_error)/dt;
    // P
    double P = current_error;

    // I
    double I = sum_error/Ti;

    // D
    double D = Td*current_deriv_error;

    // sum up to get new w
    control = Kp*(P + I + D);// + I + D);

    // update the variable
    previous_error = current_error;
    previous_deriv_error = current_deriv_error;
  };

  void pid_config_callback(wall_following_assignment::PID_parasConfig &config, uint32_t level)
  {
    Kp = config.Kp;
    Td = config.Td;
    Ti = config.Ti;
    dt = config.dt;
  }

  double get_control() {
    return control;
  };

 private:
  double Kp;
  double Td;
  double Ti;

  double current_error;
  double previous_error;

  double sum_error;
  
  double current_deriv_error;
  double previous_deriv_error;
  double control;
  double dt;
};

#endif
