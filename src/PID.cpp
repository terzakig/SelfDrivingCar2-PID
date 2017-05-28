#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/
 
PID::PID(int ntwiddle_batch) : Flag_Initialized(false), 
			       twiddleme(gain_p),
			       twiddle_winsize(ntwiddle_batch) { 
for (int i = 0; i < NUM_PIDGAINS; i++) {
 errors.push_back(0);
 K.push_back(0);
}
dp[0]=.1; dp[1] = dp[2] = .001; 
				 

} 
 
PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki, bool adapting) {
  
  K[gain_p] = Kp;
  K[gain_d] = Kd;
  K[gain_i] = Ki;
  
  cte_1_ = 0;
  
  Flag_Adapting = adapting;
  Flag_Initialized = true;
  batch_counter = 0;
  // twiddle state vars
  state = st_IDLE;
  
  
  // twiddle errors
  avg_cte = 0;
  best_avg_cte = 99999999;
}

void PID::UpdateError(double cte) {
  
  assert(Flag_Initialized && "Must run init() first!!!");
  
  // if the controller is adapting,
  // we need to twiddle the weights
  cte_ = cte; // update cte
  
  // store errors
  errors[gain_p] = cte_;
  errors[gain_d] = cte_ - cte_1_;  cte_1_ = cte_;
  errors[gain_i] += cte_;
    
  // now run twiddle...
  if (Flag_Adapting) {
    
    batch_counter++;
    avg_cte += fabs(cte_);
    
    // Twiddling!
    if (batch_counter == twiddle_winsize ) {
     
      avg_cte /= twiddle_winsize;
      //cout <<"Entering twiddle. Average error : "<<avg_cte<<endl;
      //cout <<"Old gains : "<<K[0]<<" , "<<K[1]<<" , "<<K[2]<<endl;
      Twiddle();
      //cout <<"New gains : "<<K[0]<<" , "<<K[1]<<" , "<<K[2]<<endl;
      avg_cte = 0;
      batch_counter = 0;
      
    }
    
    
  }
    
  
 
}


void PID::Twiddle() {
  
  // if the cte has decreased, store the best gains
  if (avg_cte < best_avg_cte) {
    
    best_avg_cte = avg_cte;
    // increase the search scope, if we did something to "dp" previously
    dp[twiddleme] *= 1.1;
    // great, now move to another component
    twiddleme = (PID_GAIN)( ( (int)twiddleme + 1 ) % 3 );  
    K[twiddleme] += dp[twiddleme];
    state = st_INCREASED;
  }
  else {
    if (state == st_INCREASED) {
      K[twiddleme] -= 2 * dp[twiddleme]; 
      state = st_DECREASED;
    }
    else if (state == st_DECREASED ) {
      K[twiddleme] += dp[twiddleme]; // reinstate the original gain!
      dp[twiddleme] *= 0.9;
      
      // great, now move to another component
      twiddleme = (PID_GAIN)( ( (int)twiddleme + 1 ) % 3 );  
      K[twiddleme] += dp[twiddleme];
      state = st_INCREASED;
    }
    else /* if (state == st_IDLE)*/ {
      dp[twiddleme] *= 0.9;
      // great, now move to another component
      twiddleme = (PID_GAIN)( ( (int)twiddleme + 1 ) % 3 );  
      K[twiddleme] += dp[twiddleme];
      state = st_INCREASED;
     }
  }
  
  
}