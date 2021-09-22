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

// Attempt to avoid double-initialization of C structs; not working
// use core::mem::MaybeUninit;

use cmsis_dsp_sys::{
    arm_fir_f32, 
    arm_fir_init_f32, 
    arm_fir_instance_f32,
    arm_fir_q31,
    arm_fir_fast_q31, 
    arm_fir_init_q31, 
    arm_fir_instance_q31, 
};

// todo: DOn't forget the fast q31 FIRs.


/// Wrapper for CMSIS-DSP function `arm_fir_f32` using Rust types.
/// [arm_fir_q31 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga2f6fc6582ab4491b3ea8c038c5247ecf)
/// See docs on _f32 variant for more details, including inline code comments.
// The main difference is we use `u32` instead of `f32`.
pub fn rust_arm_fir_q31(
    input: &[i32],
    output: &mut [i32],
    filter_coeffs: &[i32],
    fir_state: &mut [i32],
    block_size: u32,
    num_taps: u16,
) {
    // todo: Confirm you can drop-in u32 for q31.
    assert!(
        input.len() == output.len(),
        "Input and output array sizes must be the same."
    );

    let num_blocks = input.len() as u32 / block_size;

    let mut s = arm_fir_instance_q31 {
        numTaps: num_taps,
        pCoeffs: filter_coeffs.as_ptr(),
        pState: fir_state.as_mut_ptr(),
    };

    unsafe {
        arm_fir_init_q31(
            &mut s,
            num_taps,
            filter_coeffs.as_ptr(),
            fir_state.as_mut_ptr(),
            block_size,
        );
    }

    for i in 0..num_blocks {
        unsafe {
            arm_fir_q31(
                &mut s,
                input[i as usize * block_size as usize..].as_ptr(),
                output[i as usize * block_size as usize..].as_mut_ptr(),
                block_size,
            );
        }
    }
}


/// Wrapper for CMSIS-DSP function `arm_fir_f32` using Rust types.
/// [arm_fir_f32 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga0cf008f650a75f5e2cf82d10691b64d9)
/// See https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIRLPF.html
/// and https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_fir_example_f32_8c-example.html
/// For example on on this code structure.
///
/// Coefficients can be generated using fir1() MATLAB function. eg fir1(28, 6/24)
pub fn rust_arm_fir_f32(
    input: &[f32],
    output: &mut [f32],
    filter_coeffs: &[f32],
    fir_state: &mut [f32],
    block_size: u32,
    num_taps: u16,
) {
    assert!(
        input.len() == output.len(),
        "Input and output array sizes must be the same."
    );

    let num_blocks = input.len() as u32 / block_size;

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__fir__instance__f32.html
    // Data Fields:
    // uint16_t 	numTaps
    // float32_t * 	pState
    // const float32_t * 	pCoeffs
    // `MaybeUnit` skips zeroizing, which is required by the C API.
    // let mut s = MaybeUninit::uninit();

    let mut s = arm_fir_instance_f32 {
        numTaps: num_taps,
        pCoeffs: filter_coeffs.as_ptr(),
        pState: fir_state.as_mut_ptr(),
    };

    // Note: We end up initializing both above, and then again in `arm_fir_init_f32`,
    // which is perhaps a duplicate.

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
    unsafe {
        arm_fir_init_f32(
            &mut s,
            num_taps,
            filter_coeffs.as_ptr(),
            fir_state.as_mut_ptr(),
            block_size,
        );
    }

    // Call the FIR process function for every blockSize samples
    for i in 0..num_blocks {
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
        unsafe {
            arm_fir_f32(
                &mut s,
                input[i as usize * block_size as usize..].as_ptr(),
                output[i as usize * block_size as usize..].as_mut_ptr(),
                block_size,
            );
        }
    }
}
