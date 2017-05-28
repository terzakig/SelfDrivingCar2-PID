#ifndef PID_H
#define PID_H

#define NUM_PIDGAINS 3

#include <assert.h>
#include <iostream>
#include <cmath>
#include <vector>

class PID {
public:
  
  enum PID_GAIN {gain_p = 0, gain_d = 1, gain_i = 2};
  enum TWIDDLE_STATE {st_INCREASED, st_DECREASED, st_IDLE};
  
  /*
  * Errors
  */
  std::vector<double> errors;
  
  /*
  * gains
  */ 
  std::vector<double> K;
 
  /*
  * Constructor
  */
  PID(int ntwiddle_batch = 5);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp = 0.3, double Kd = 0.0, double Ki = 0.0, bool adapting = true);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate PID feedback.
  */
  inline double Feedback() {
  
    double feed = 0;
    for (int i = 0; i < NUM_PIDGAINS; i++)
      feed -= K[i] * errors[i];
    
    return feed;
  }
  
  
private:
  // the twiddle heuristic!
  void Twiddle();
  // previous cte
  double cte_1_;
  // current cte
  double cte_;
  // gain currently being "twiddled"
  PID_GAIN twiddleme;
  // a flag indicating that the controller is adapting
  // (i.e., "twiddling" its gains)
  bool Flag_Adapting;
  // Indicates if the controller has been initialized
  bool Flag_Initialized;
  // The "twiddle" increements
  double dp[NUM_PIDGAINS];
  // window size for evaluating error behaviour in twidle
  int twiddle_winsize;
  // average cte current and previous errors over the twiddle window
  double avg_cte;
  double best_avg_cte;
  // The batch counter
  int batch_counter;
  // next twiddle twiddle action 
  TWIDDLE_STATE state;
  
};

#endif /* PID_H */
