/*==========================================================
 * filter.h
 *
 * Header file for filter.cpp. 
 * 
 * Implements an IIR filter:
 *          
 *           b0 + b1 * z^-1 + b2 * z^-2 + ... + bn * z^-n
 *  H(z) =  -----------------------------------------------
 *           1  + a1 * z^-1 + a2 * z^-2 + ... + an * z^-n
 *
 * Created by: Vinay Chawda
 *             vinay.chawda@disneyresearch.com
 *
 * Date created: 03/18/2016
 *
 * Last modified: 03/18/2016
 *========================================================*/

#ifndef KINEMATICS__IIR_FILTER_H_
#define KINEMATICS__IIR_FILTER_H_

#define FILTER_ORDER 1      					// #define filter order to avoid dynamic memory allocations

class IirFilter {
    public: 
        IirFilter();
        IirFilter(double *b, double *a);
        ~IirFilter();
        void ResetFilter();
        void SetFilter(double *b, double *a);
        double Filter(double in);

    private:
        double b_[FILTER_ORDER+1];               	// Numerator coefficients
        double a_[FILTER_ORDER+1];               	// Denominator coefficients. a[0] should be 1 
        double input_buffer_[FILTER_ORDER+1];       // Buffer to store input signal values
        double output_buffer_[FILTER_ORDER+1];      // Buffer to store output signal values  

};

#endif // KINEMATICS__IIR_FILTER_H_
