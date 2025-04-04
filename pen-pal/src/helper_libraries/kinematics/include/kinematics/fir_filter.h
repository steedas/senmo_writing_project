/** ============================================================================
 *  @file    fir_filter.h
 *  @author  Matthew Pan (matthew.pan@queensu.ca)
 *  @date    June 30, 2023 
 *  @version 1.0 
 *  
 *  @brief Header file to implement an FIR filter
 *
 *  @section Header file to implement a finite impulse response (FIR) filter. 
 *  Recall that the output of a FIR filter is the weight sum of the N most 
 *  recent input values.
 * 
 *  y[n] = b_0*x[n] + b_1*x[n-1] + ... + b_N*x[n-N]
 * 
 *  where: 
 *  	- x[n] is the input signal
 * 		- y[n] is the output signal
 * 		- N is the filter order
 *  	- b_i is the value of the inpulse reponse at the ith instant for 0<=i<=N 
 * 		  of an Nth order filter 
 *  
 * ===========================================================================*/

#ifndef KINEMATICS__FIR_FILTER_H_
#define KINEMATICS__FIR_FILTER_H_

#include <algorithm>
#include <string.h>
#define BUFFER_SIZE 10000

class FirFilter {
    public: 
        int taps_;
        double *coeffs_;

        FirFilter(int taps, double coeffs[]);
        FirFilter(int taps, double coeff);
        ~FirFilter();
        void SetFilter(int taps, double coeffs[]);
        void SetFilter(int taps, double coeff);
        void ResetFilter();
        double Filter(double in);

    private:
        double input_buffer_[BUFFER_SIZE];
        double output_buffer_[BUFFER_SIZE];
        int offset_;
};

#endif // KINEMATICS__FIR_FILTER_H_
