#ifndef MAX30102_APPROX_H_INCLUDED
#define MAX30102_APPROX_H_INCLUDED

/*
 * Headers file for the approximation of the R to SpO2(%) for the optical
 * sensor based on the MAX30102.
 */

// Approximation of SpO2 percentage from R value
#ifdef MAX30102_APPROX_TABLE
// The approximation table uses: SpO2(%) = 1.5958422*R^2 -34.6596622*R + 112.6898759 equation
// from MAX30101 sensor calibration without optical shield,
// APPLICATION NOTE 6845
// GUIDELINES FOR SPO2 MEASUREMENT USING THE MAXIM MAX32664 SENSOR HUB.
unsigned char SpO2_percent[] = {
    99, 97, 96, 94, 92, 91, 89, 88, 86, 84,
    83, 81, 80, 78, 76, 75, 73, 72, 70, 69,
    67, 66, 64, 63, 61, 60, 58, 57, 55, 54,
    53, 51, 50, 48, 47, 46, 44, 43, 41, 40,
    39, 37, 36, 35, 33, 32, 31, 29, 28, 27,
    26, 24, 23, 22, 21
}; // R = 0.4 to 3.1, step 0.05; idx = 20 * R - 8; idx = 0 to 54
// Approximation coefficients for index calculation from the table
#define K_IDX 20
#define B_IDX -8
#endif

// Linear approximation of SpO2_percent table in range 99% to 31% (R = 0.4 to 2.7, idx = 0 to 46)
// SpO2(%) = -30 * R + 110
#define K_APPROX_SpO2 30 // "-"
#define B_APPROX_SpO2 110
#define TOP_APPROX (B_APPROX_SpO2 - 99) // <= 99%
#define BOTTOM_APPROX (B_APPROX_SpO2 - 31) // <= 31%

#endif // MAX30102_APPROX_H_INCLUDED
