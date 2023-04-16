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

// todo: Consider having wrapper structs that impl Send (for use with RTIC etc),
// todo, and own their state and coefficient buffers!

use core::{
    mem::MaybeUninit,
    sync::atomic::{compiler_fence, Ordering},
};

use cmsis_dsp_sys as sys;

/// Wrapper for CMSIS-DSP function `arm_fir_q31` using Rust types.
/// [arm_fir_q31 docs](https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR.html#ga2f6fc6582ab4491b3ea8c038c5247ecf)
/// See docs on _q31 variant for more details, including inline code comments.
// The main difference is we use `i32` instead of `q31`.
/// Wrapper for CMSIS-DSP function `arm_fir_init_q31`.
pub fn fir_init_q31(
    s: &mut sys::arm_fir_instance_q31,
    filter_coeffs: &'static [i32],
    state: &'static mut [i32],
    block_size: usize,
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
            block_size as u32,
        );
    }
}

/// Initialize an empty instance of `arm_fir_instance_q31`. Used for setting up static types.
/// See https://www.keil.com/pack/doc/CMSIS/DSP/html/structarm__fir__decimate__instance__f32.html
/// The CMSIS-DSP C-API requires us to initialize, eg with 0s.
pub fn fir_init_empty_q31() -> sys::arm_fir_instance_q31 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_fir_instance_q31 {
        numTaps: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
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
    block_size: usize,
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
        sys::arm_fir_q31(s, input.as_ptr(), output.as_mut_ptr(), block_size as u32);
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_init_f32`.
pub fn fir_init_f32(
    s: &mut sys::arm_fir_instance_f32,
    filter_coeffs: &'static [f32],
    state: &'static mut [f32],
    block_size: usize,
) {
    let num_taps = filter_coeffs.len();
    // pState points to the array of state variables. pState is of length numTaps+blockSize-1 samples
    // where blockSize is the number of input samples processed by each call to arm_fir_f32().
    assert_eq!(state.len(), num_taps + block_size - 1);

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
            block_size as u32,
        );
    }
}

/// Initialize an empty instance of `arm_fir_instance_f32`. Used for setting up static types.
pub fn fir_init_empty_f32() -> sys::arm_fir_instance_f32 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_fir_instance_f32 {
        numTaps: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
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
    block_size: usize,
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
        sys::arm_fir_f32(s, input.as_ptr(), output.as_mut_ptr(), block_size as u32);
    }
}
//
// /// Used to satisfy RTIC resource Send requirements. Owns its state and buffer, so you don't
// /// have to use statics.
// pub struct FirInstF32 {
//     pub inst: sys::arm_fir_instance_f32,
//     pub coeffs: &'static [f32],
//     pub state: &'static mut [f32],
// }
//
// unsafe impl Send for FirF32Inst {}
//
// impl FirInstF32 {
//     pub fn new(coeffs: &'static [f32], state: &'static mut [f32], block_size: usize) -> Self {
//         let mut inst = fir_init_empty_f32();
//         Self {
//             inst: fir_init_f32(&mut inst, coeffs, state, block_size);
//             coeffs,
//             state,
//         }
//     }
// }
//
// /// Used to satisfy RTIC resource Send requirements. Owns its state and buffer, so you don't
// /// have to use statics.
// pub struct IirInstF32 {
//     pub inst: sys::arm_biquad_casd_df1_inst_f32,
//     pub coeffs: &'static [f32],
//     pub state: &'static mut [f32],
// }
//
// unsafe impl Send for IirF32Inst {}

// impl FirInstF32 {
//     pub fn new() -> Self {
//         Self {
//             inst: fir_in_f32(),
//         }
//     }
// }

/// Wrapper for CMSIS-DSP function `arm_fir_decimate_init_f32`.
pub fn fir_decimate_init_f32(
    s: &mut sys::arm_fir_decimate_instance_f32,
    decimation_factor: u8,
    filter_coeffs: &'static [f32],
    state: &'static mut [f32],
    block_size: usize,
) {
    let num_taps = filter_coeffs.len();
    // pState points to the array of state variables. pState is of length numTaps+blockSize-1 samples
    // where blockSize is the number of input samples processed by each call to arm_fir_f32().
    assert_eq!(state.len(), num_taps + block_size - 1);

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR__decimate.html#ga67c80582fc296552fef2bd1f208d853b
    // Parameters:
    // [in,out]	S	points to an instance of the floating-point FIR decimator structure
    // [in]	numTaps	number of coefficients in the filter
    // [in]	M	decimation factor
    // [in]	pCoeffs	points to the filter coefficients
    // [in]	pState	points to the state buffer
    // [in]	blockSize	number of input samples to process per call

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/arm__fir__decimate__init__f32_8c.html
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_decimate_init_f32(
            s,
            num_taps as u16,
            decimation_factor,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            block_size as u32,
        );
    }
}

/// Initialize an empty instance of `arm_fir_decimate_instance_f32`. Used for setting up static types.
pub fn fir_decimate_init_empty_f32() -> sys::arm_fir_decimate_instance_f32 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_fir_decimate_instance_f32 {
        M: 0,
        numTaps: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_decimate_f32` using Rust types.
/// The FIR decimator functions provided in the CMSIS DSP Library combine the FIR filter
/// and the decimator in an efficient manner. Instead of calculating all of the FIR filter
/// outputs and discarding M-1 out of every M, only the samples output by the decimator are
/// computed. The functions operate on blocks of input and output data. pSrc points to an array
/// of blockSize input values and pDst points to an array of blockSize/M output values.
/// In order to have an integer number of output samples blockSize must always be a multiple
/// of the decimation factor M.
pub fn fir_decimate_f32(
    s: &mut sys::arm_fir_decimate_instance_f32,
    input: &[f32],
    output: &mut [f32],
    block_size: usize,
) {
    // void arm_fir_decimate_f32 	( 	const arm_fir_decimate_instance_f32 *  	S,
    // 		const float32_t *  	pSrc,
    // 		float32_t *  	pDst,
    // 		uint32_t  	blockSize
    // 	)
    //
    // [in]	S	points to an instance of the floating-point FIR decimator structure
    // [in]	pSrc	points to the block of input data
    // [out]	pDst	points to the block of output data
    // [in]	blockSize	number of samples to process
    // Returns none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_decimate_f32(s, input.as_ptr(), output.as_mut_ptr(), block_size as u32);
    }
}

/// Wrapper for CMSIS-DSP function `arm_fir_interpolate_init_f32`.
/// `block_size` is the number if input samples processed. (Not output).
pub fn fir_interpolate_init_f32(
    s: &mut sys::arm_fir_interpolate_instance_f32,
    upsample_factor: u8,
    filter_coeffs: &'static [f32],
    state: &'static mut [f32],
    block_size: usize,
) {
    let num_taps = filter_coeffs.len();
    // pState points to the array of state variables. pState is of length (numTaps/L)+blockSize-1 words
    // where blockSize is the number of input samples processed by each call to arm_fir_interpolate_f32().
    assert_eq!(
        state.len(),
        num_taps / upsample_factor as usize + block_size - 1
    );

    // https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR__Interpolate.html#ga1416bcc1bcf6e2b18ff15261b6e04345
    // arm_status arm_fir_interpolate_init_f32 	( 	arm_fir_interpolate_instance_f32 *  	S,
    // 		uint8_t  	L,
    // 		uint16_t  	numTaps,
    // 		const float32_t *  	pCoeffs,
    // 		float32_t *  	pState,
    // 		uint32_t  	blockSize
    // 	)
    //
    // Parameters
    //     [in,out]	S	points to an instance of the floating-point FIR interpolator structure
    //     [in]	L	upsample factor
    //     [in]	numTaps	number of filter coefficients in the filter
    //     [in]	pCoeffs	points to the filter coefficient buffer
    //     [in]	pState	points to the state buffer
    //     [in]	blockSize	number of input samples to process per call
    //
    // Returns
    //     execution status
    //
    //         ARM_MATH_SUCCESS : Operation successful
    //         ARM_MATH_ARGUMENT_ERROR : filter length numTaps is not a multiple of the interpolation factor L

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_interpolate_init_f32(
            s,
            upsample_factor,
            num_taps as u16,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            block_size as u32,
        );
    }
}

/// Initialize an empty instance of `arm_fir_interpolate_instance_f32`. Used for setting up static types.
pub fn fir_interpolate_init_empty_f32() -> sys::arm_fir_interpolate_instance_f32 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_fir_interpolate_instance_f32 {
        L: 0,
        phaseLength: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
    }
}

/// Wrapper for CMSIS-DSP function `arm_interplate_f32`.
/// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__FIR__Interpolate.html
pub fn fir_interpolate_f32(
    s: &mut sys::arm_fir_interpolate_instance_f32,
    input: &[f32],
    output: &mut [f32],
    block_size: usize,
) {
    // void arm_fir_interpolate_f32 	( 	const arm_fir_interpolate_instance_f32 *  	S,
    // 		const float32_t *  	pSrc,
    // 		float32_t *  	pDst,
    // 		uint32_t  	blockSize
    // 	)

    // Processing function for the floating-point FIR interpolator.
    //
    // Parameters
    //     [in]	S	points to an instance of the floating-point FIR interpolator structure
    //     [in]	pSrc	points to the block of input data
    //     [out]	pDst	points to the block of output data
    //     [in]	blockSize	number of samples to process
    //
    // Returns
    //     none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_fir_interpolate_f32(s, input.as_ptr(), output.as_mut_ptr(), block_size as u32);
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_init_q15`.
pub fn biquad_cascade_df1_init_q15(
    s: &mut sys::arm_biquad_casd_df1_inst_q15,
    filter_coeffs: &'static [i16],
    state: &'static mut [i16],
    post_shift: i8,
) {
    let num_stages = filter_coeffs.len() / 5;

    // The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values.
    assert!(state.len() == 4 * num_stages);

    // See notes for f32 variant.
    // Parameters
    //     [in,out]	S	points to an instance of the Q31 Biquad cascade structure.
    //     [in]	numStages	number of 2nd order stages in the filter.
    //     [in]	pCoeffs	points to the filter coefficients.
    //     [in]	pState	points to the state buffer.
    //     [in]	postShift	Shift to be applied after the accumulator. Varies according to the coefficients format

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_init_q15(
            s,
            num_stages as u8,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            post_shift,
        );
    }
}

/// Initialize an empty instance of `arm_biquad_casd_df1_inst_q31`. Used for setting up static types.
pub fn biquad_cascade_df1_init_empty_q15() -> sys::arm_biquad_casd_df1_inst_q15 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_biquad_casd_df1_inst_q15 {
        numStages: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
        postShift: 0,
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_q15`.
pub fn biquad_cascade_df1_q15(
    s: &mut sys::arm_biquad_casd_df1_inst_q15,
    input: &[i16],
    output: &mut [i16],
    block_size: u32,
) {
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_q15(s, input.as_ptr(), output.as_mut_ptr(), block_size);
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_init_q31`.
pub fn biquad_cascade_df1_init_q31(
    s: &mut sys::arm_biquad_casd_df1_inst_q31,
    filter_coeffs: &'static [i32],
    state: &'static mut [i32],
    post_shift: i8,
) {
    let num_stages = filter_coeffs.len() / 5;

    // The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values.
    assert!(state.len() == 4 * num_stages);

    // See notes for f32 variant.
    // Parameters
    //     [in,out]	S	points to an instance of the Q31 Biquad cascade structure.
    //     [in]	numStages	number of 2nd order stages in the filter.
    //     [in]	pCoeffs	points to the filter coefficients.
    //     [in]	pState	points to the state buffer.
    //     [in]	postShift	Shift to be applied after the accumulator. Varies according to the coefficients format

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_init_q31(
            s,
            num_stages as u8,
            filter_coeffs.as_ptr(),
            state.as_mut_ptr(),
            post_shift,
        );
    }
}

/// Initialize an empty instance of `arm_biquad_casd_df1_inst_q31`. Used for setting up static types.
pub fn biquad_cascade_df1_init_empty_q31() -> sys::arm_biquad_casd_df1_inst_q31 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_biquad_casd_df1_inst_q31 {
        numStages: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
        postShift: 0,
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_q31`.
pub fn biquad_cascade_df1_q31(
    s: &mut sys::arm_biquad_casd_df1_inst_q31,
    input: &[i32],
    output: &mut [i32],
    block_size: u32,
) {
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_biquad_cascade_df1_q31(s, input.as_ptr(), output.as_mut_ptr(), block_size);
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df1_init_f32`.
pub fn biquad_cascade_df1_init_f32(
    s: &mut sys::arm_biquad_casd_df1_inst_f32,
    filter_coeffs: &'static [f32],
    state: &'static mut [f32],
) {
    let num_stages = filter_coeffs.len() / 5;

    // The 4 state variables for stage 1 are first, then the 4 state variables for stage 2, and so on.
    // The state array has a total length of 4*numStages values.
    assert!(state.len() == 4 * num_stages);

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

/// Initialize an empty instance of `arm_biquad_casd_df1_inst_f32`. Used for setting up static types.
pub fn biquad_cascade_df1_init_empty_f32() -> sys::arm_biquad_casd_df1_inst_f32 {
    let mut uninit_ptr = MaybeUninit::uninit();

    sys::arm_biquad_casd_df1_inst_f32 {
        numStages: 0,
        pCoeffs: uninit_ptr.as_ptr(),
        pState: uninit_ptr.as_mut_ptr(),
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
        sys::arm_biquad_cascade_df1_f32(s, input.as_ptr(), output.as_mut_ptr(), block_size);
    }
}

/// Wrapper for CMSIS-DSP function `arm_biquad_cascade_df2T_init_f32`.
pub fn biquad_cascade_df2T_init_f32(
    s: &mut sys::arm_biquad_cascade_df2T_instance_f32,
    filter_coeffs: &'static [f32],
    state: &'static mut [f32],
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
        sys::arm_biquad_cascade_df2T_f32(s, input.as_ptr(), output.as_mut_ptr(), block_size);
    }
}

/// Wrapper for CMSIS-DSP function `arm_rfft_fast_init_f32`.
/// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html#gac5fceb172551e7c11eb4d0e17ef15aa3
pub fn rfft_fast_init_f32(s: &mut sys::arm_rfft_fast_instance_f32, fft_len: u16) {
    // Parameters
    //     [in,out]	S	points to an arm_rfft_fast_instance_f32 structure
    //     [in]	fftLen	length of the Real Sequence

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_rfft_fast_init_f32(s, fft_len);
    }
}

/// Wrapper for CMSIS-DSP function `arm_rfft_fast_f32`.
/// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html#ga5d2ec62f3e35575eba467d09ddcd98b5
/// Note: The input buffer is modified by this function, as documented here.
pub fn rfft_fast_f32(
    s: &mut sys::arm_rfft_fast_instance_f32,
    input: &mut [f32],
    output: &mut [f32],
    ifft: bool,
) {
    // Parameters
    //     [in]	S	points to an arm_rfft_fast_instance_f32 structure
    //     [in]	p	points to input buffer (Source buffer is modified by this function.)
    //     [in]	pOut	points to output buffer
    //     [in]	ifftFlag
    //
    //         value = 0: RFFT
    //         value = 1: RIFFT

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_rfft_fast_f32(
            s,
            input.as_mut_ptr(),
            output.as_mut_ptr(),
            if ifft == true { 1 } else { 0 },
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_correlate_f32`.
/// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__Corr.html#ga371054f6e5fd78bec668908251f1b2f2
pub fn correlate_f32(
    src_a: &[f32],
    src_a_len: usize,
    src_b: &[f32],
    src_b_len: usize,
    p_dist: &mut [f32],
) {
    // Parameters
    //     [in]	pSrcA	points to the first input sequence
    //     [in]	srcALen	length of the first input sequence
    //     [in]	pSrcB	points to the second input sequence
    //     [in]	srcBLen	length of the second input sequence
    //     [out]	pDst	points to the location where the output result is written. Length 2 * max(srcALen, srcBLen) - 1.
    //
    // Returns
    //     none

    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_correlate_f32(
            src_a.as_ptr(),
            src_a_len as u32,
            src_b.as_ptr(),
            src_b_len as u32,
            p_dist.as_mut_ptr(),
        );
    }
}

/// Wrapper for CMSIS-DSP function `arm_correlation_distance_f32`.
/// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__Correlation.html#gaf51cef11ade667912bb004cb24dc4e39
pub fn correlation_distance_f32(p_a: &mut [f32], p_b: &mut [f32], block_size: usize) -> f32 {
    compiler_fence(Ordering::SeqCst);
    unsafe {
        sys::arm_correlation_distance_f32(p_a.as_mut_ptr(), p_b.as_mut_ptr(), block_size as u32)
    }
}
