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

use cmsis_dsp_sys::{arm_fir_f32, arm_fir_init_f32, arm_fir_instance_f32};


/// Wrapper for CMSIS-DSP function `arm_fir_f32` using Rust types.
/// [arm_fir_f32 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga0cf008f650a75f5e2cf82d10691b64d9)
/// See https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIRLPF.html
/// and https://www.keil.com/pack/doc/CMSIS/DSP/html/arm_fir_example_f32_8c-example.html
/// For example on on this code structure.
/// 
/// Coefficients can be generated using fir1() MATLAB function. eg fir1(28, 6/24)
pub fn rust_arm_fir_f32(
    input: &[f32], 
    output: &mut[f32],
    coeffs: &[f32],
    nr_threshold: f32,
    block_size: u32,
    num_taps: u16,
) {
    assert!(input.len() == output.len(), "Input and output array sizes must be the same.");

    let inputF32 = input.as_ptr();
    let mut outputF32 = output.as_mut_ptr();

    // Declare State buffer of size (numTaps + blockSize - 1)
    // let mut fir_state = [0.; (num_taps as u32 + block_size - 1) as usize];
    let mut fir_state = [0.; 80]; // todo TS

    let num_blocks = input.len() as u32 / block_size;

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__fir__instance__f32.html
    // Data Fields:
    // uint16_t 	numTaps
    // float32_t * 	pState
    // const float32_t * 	pCoeffs
    // todo: In the official example, this isn't initialized directly.
    // todo: is there a way to do that here?
    let mut S = arm_fir_instance_f32 {
        numTaps: num_taps,
        pCoeffs: coeffs.as_ptr(),
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
            &mut S,
            num_taps,
            coeffs.as_ptr(),
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
                &mut S,
                input[i as usize * block_size as usize..].as_ptr(),
                output[i as usize * block_size as usize..].as_mut_ptr(),
                block_size,
            );
        }
    }
}