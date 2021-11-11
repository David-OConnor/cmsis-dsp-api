#![no_std]
// We may use DSP terms that doesn't follow Rust naming conventions.
#![allow(non_snake_case)]

//! This library provides a Rust API for wrapped CMSIS functions, eg using Jacob
//! Rosenthall's [CMSIS-DSP-SYS](https://github.com/jacobrosenthal/cmsis-dsp-sys) library.
//! Can be adapted to any library that similarly uses Bindgen to wrap CMSIS-DISP, by
//! changing the dependency in Cargo.toml.
//!
//! The bindgen-generated code uses pointers, which have safety concerns, and a non-ideal
//! API. This library wraps these, exposing an API with Rust arrays and references.
//!
//! Only wraps functions that map to C pointers; these are the ones that need an API improvement.
//! For ones that don't (Eg arm_sin_), use the wrapped CMSIS library directly.

use core::sync::atomic::{compiler_fence, Ordering};

use cmsis_dsp_sys as sys;

/// Wrapper for CMSIS-DSP function `arm_fir_q31` using Rust types.
/// [arm_fir_q31 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga2f6fc6582ab4491b3ea8c038c5247ecf)
/// See docs on _q31 variant for more details, including inline code comments.
// The main difference is we use `i32` instead of `q31`.
/// Wrapper for CMSIS-DSP function `arm_fir_init_q31`.
pub fn fir_init_q31(
    s: &mut sys::arm_fir_instance_q31,
    filter_coeffs: &[i32],
    state: &mut [i32],
    block_size: u32,
) {
    let num_taps = filter_coeffs.len();
    // pState points to the array of state variables. pState is of length numTaps+blockSize-1 samples
    // where blockSize is the number of input samples processed by each call to arm_fir_q31().
    assert_eq!(state.len(), num_taps + block_size as usize - 1);

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__fir__instance__q31.html
    // Data Fields:
    // uint16_t 	numTaps
    // float32_t * 	pState
    // const float32_t * 	pCoeffs

    // Call FIR init function to initialize the instance structure.
    // void arm_fir_init_q31 	(
    //      arm_fir_instance_q31 *  	S,
    //	    uint16_t  	numTaps,
    //	    const float32_t *  	pCoeffs,
    //	    float32_t *  	pState,
    //	    uint32_t  	blockSize
    // )
    // Parameters
    // [in,out]	S	points to an instance of the floating-point FIR filter structure
    // [in]	numTaps	number of filter coefficients in the filter
    // [in]	pCoeffs	points to the filter coefficients buffer
    // [in]	pState	points to the state buffer
    // [in]	blockSize	number of samples processed per call
    // Returns none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_init_q31(
            s,
            num_taps as u16,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            block_size,
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_q31` using Rust types.
/// [arm_fir_q31 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga0cf008f650a75f5e2cf82d10691b64d9)
/// See https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIRLPF.html
/// and https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_fir_example_q31_8c-example.html
/// For example on on this code structure.
///
/// Coefficients can be generated using fir1() MATLAB function. eg fir1(28, 6/24)
pub fn fir_q31(
    s: &mut sys::arm_fir_instance_q31,
    input: &[i32],
    output: &mut [i32],
    block_size: u32,
) {
    // void arm_fir_q31 	(
    //     const arm_fir_instance_q31 *  	S,
    //     const float32_t *  	pSrc,
    //     float32_t *  	pDst,
    //     uint32_t  	blockSize
    // )
    // Parameters
    // [in]	S	points to an instance of the floating-point FIR filter structure
    // [in]	pSrc	points to the block of input data
    // [out]	pDst	points to the block of output data
    // [in]	blockSize	number of samples to process
    // Returns none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_q31(
            s,
            input.as_ptr(),
            output.as_mut_ptr(),
            block_size,
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_init_f32`.
pub fn fir_init_f32(
    s: &mut sys::arm_fir_instance_f32,
    filter_coeffs: &[f32],
    state: &mut [f32],
    block_size: u32,
) {
    let num_taps = filter_coeffs.len();
    // pState points to the array of state variables. pState is of length numTaps+blockSize-1 samples
    // where blockSize is the number of input samples processed by each call to arm_fir_f32().
    assert_eq!(state.len(), num_taps + block_size as usize - 1);

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__fir__instance__f32.html
    // Data Fields:
    // uint16_t 	numTaps
    // float32_t * 	pState
    // const float32_t * 	pCoeffs

    // Call FIR init function to initialize the instance structure.
    // void arm_fir_init_f32 	(
    //      arm_fir_instance_f32 *  	S,
    //	    uint16_t  	numTaps,
    //	    const float32_t *  	pCoeffs,
    //	    float32_t *  	pState,
    //	    uint32_t  	blockSize
    // )
    // Parameters
    // [in,out]	S	points to an instance of the floating-point FIR filter structure
    // [in]	numTaps	number of filter coefficients in the filter
    // [in]	pCoeffs	points to the filter coefficients buffer
    // [in]	pState	points to the state buffer
    // [in]	blockSize	number of samples processed per call
    // Returns none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_init_f32(
            s,
            num_taps as u16,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            block_size,
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_f32` using Rust types.
/// [arm_fir_f32 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga0cf008f650a75f5e2cf82d10691b64d9)
/// See https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIRLPF.html
/// and https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_fir_example_f32_8c-example.html
/// For example on on this code structure.
///
/// Coefficients can be generated using fir1() MATLAB function. eg fir1(28, 6/24)
pub fn fir_f32(
    s: &mut sys::arm_fir_instance_f32,
    input: &[f32],
    output: &mut [f32],
    block_size: u32,
) {
    // void arm_fir_f32 	(
    //     const arm_fir_instance_f32 *  	S,
    //     const float32_t *  	pSrc,
    //     float32_t *  	pDst,
    //     uint32_t  	blockSize
    // )
    // Parameters
    // [in]	S	points to an instance of the floating-point FIR filter structure
    // [in]	pSrc	points to the block of input data
    // [out]	pDst	points to the block of output data
    // [in]	blockSize	number of samples to process
    // Returns none
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_f32(
            s,
            input.as_ptr(),
            output.as_mut_ptr(),
            block_size,
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_init_f32`.
pub fn biquad_cascade_df1_init_f32(
    s: &mut sys::arm_biquad_casd_df1_inst_f32,
    filter_coeffs: &[f32],
    state: &mut [f32],
) {
    let num_stages = filter_coeffs.len() / 5;

    // The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values.
    assert!(state.len() == 4 * num_stages);

    // let mut s = sys::arm_biquad_casd_df1_inst_f32 {
    //     numStages: num_stages,
    //     pCoeffs: filter_coeffs.as_ptr(),
    //     pState: state.as_mut_ptr(),
    // };

    //     The coefficients are stored in the array pCoeffs in the following order:
    //
    //         {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
    //
    //     where b1x and a1x are the coefficients for the first stage, b2x and a2x are the coefficients
    // for the second stage, and so on. The pCoeffs array contains a total of 5*numStages values.
    //
    //     The pState is a pointer to state array. Each Biquad stage has 4 state variables x[n-1], x[n-2],
    // y[n-1], and y[n-2]. The state variables are arranged in the pState array as:
    //
    //         {x[n-1], x[n-2], y[n-1], y[n-2]}
    //
    //     The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values. The state variables are updated after
    // each block of data is processed; the coefficients are untouched.

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_init_f32(
            s,
            num_stages as u8,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_f32`.
pub fn biquad_cascade_df1_f32(
    s: &mut sys::arm_biquad_casd_df1_inst_f32,
    input: &[f32],
    output: &mut [f32],
    block_size: u32,
) {
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_f32(
            s,
            input.as_ptr(),
            output.as_mut_ptr(),
            block_size,
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df2T_init_f32`.
pub fn biquad_cascade_df2T_init_f32(
    s: &mut sys::arm_biquad_cascade_df2T_instance_f32,
    filter_coeffs: &[f32],
    state: &mut [f32],
) {
    let num_stages = filter_coeffs.len() / 5;

    // The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values.
    assert!(state.len() == 4 * num_stages);

    // let mut s = sys::arm_biquad_casd_df2T_inst_f32 {
    //     numStages: num_stages,
    //     pCoeffs: filter_coeffs.as_ptr(),
    //     pState: state.as_mut_ptr(),
    // };

    //     The coefficients are stored in the array pCoeffs in the following order:
    //
    //         {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
    //
    //     where b1x and a1x are the coefficients for the first stage, b2x and a2x are the coefficients
    // for the second stage, and so on. The pCoeffs array contains a total of 5*numStages values.
    //
    //     The pState is a pointer to state array. Each Biquad stage has 4 state variables x[n-1], x[n-2],
    // y[n-1], and y[n-2]. The state variables are arranged in the pState array as:
    //
    //         {x[n-1], x[n-2], y[n-1], y[n-2]}
    //
    //     The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values. The state variables are updated after
    // each block of data is processed; the coefficients are untouched.

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df2T_init_f32(
            s,
            num_stages as u8,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df2T_f32`.
pub fn biquad_cascade_df2T_f32(
    s: &mut sys::arm_biquad_cascade_df2T_instance_f32,
    input: &[f32],
    output: &mut [f32],
    block_size: u32,
) {

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df2T_f32(
            s,
            input.as_ptr(),
            output.as_mut_ptr(),
            block_size,
        );
    }
}
