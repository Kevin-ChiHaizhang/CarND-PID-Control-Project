#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;

class TWIDDLE {
public:

  // Vector of parameters in scale of change
  std::vector<double> params;
  std::vector<double> d_params;

  // Best error
  double best_error;

  // Tolerance used to stop
  double tolerance;

  // NUmber of parameters to tune
  int n_parameters;

  // Used to control in which part of the algorithm we are
  int auxiliar;

  // Used to control which parameter is being tuned
  int idx_control;

  bool initialized;

    /*
  * Constructor
  */
  TWIDDLE();

  /*
  * Destructor.
  */
  virtual ~TWIDDLE();

  /*
  * Initialize the optimizer
  */
  void Init(double tol, int num_param);

  /*
  * Returns a set of parameters to test given the error in the last trial
  */
  std::vector<double> get_Parameters();

  void UpdateError(double error);

  /*
  * Returns true if the parameters have already been tuned
  */
  bool is_Over();
};


TWIDDLE::TWIDDLE() {}

TWIDDLE::~TWIDDLE() {}

void TWIDDLE::Init(double tol, int num_param){
  tolerance = tol;
  n_parameters = num_param;

  d_params.push_back(1.0);
  d_params.push_back(1.0);
  d_params.push_back(0.01);

  // These initial values were found manually
  params.push_back(1.0);
  params.push_back(15.0);
  params.push_back(0.01);

  initialized = false;

  idx_control = 0;
  best_error = numeric_limits<double>::max();
  auxiliar = 0;
}

void TWIDDLE::UpdateError(double error){
  if(!initialized){
    initialized = true;
    best_error = error;
  }

  if(auxiliar == 0){
    params[idx_control] += d_params[idx_control];
    auxiliar += 1;
  }
  else{
    if(auxiliar == 1){
      if(error < best_error){
        best_error = error;
        d_params[idx_control] *= 1.1;

        auxiliar = 0;

        // Cycle parameters
        idx_control = (idx_control+1) % n_parameters;

        UpdateError(0.0); // Update parameters before leaving function
      }
      else{
        params[idx_control] -= 2*d_params[idx_control];
        auxiliar += 1;
      }
    }
    else{
      if(error < best_error){
        best_error = error;
        d_params[idx_control] *= 1.1;
      }
      else{
        params[idx_control] += d_params[idx_control];
        d_params[idx_control] *= 0.9;
      }
      auxiliar = 0;

      // Cycle parameters
      idx_control = (idx_control+1) % n_parameters;
      UpdateError(0.0); // Update parameters before leaving function
    }
  }
}

std::vector<double> TWIDDLE::get_Parameters(){
  return params;
}

bool TWIDDLE::is_Over(){
    
  double sum_dp = 0.0;
  for(int i=0; i<n_parameters; ++i){
    sum_dp += d_params[i];
  }
  return (sum_dp < tolerance);
}

#endif /* TWIDDLE_H */
