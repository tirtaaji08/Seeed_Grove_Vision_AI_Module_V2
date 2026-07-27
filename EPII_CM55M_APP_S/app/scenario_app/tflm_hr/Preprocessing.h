#ifndef Preprocessing_H
#define Preprocessing_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Definisi ukuran
#define WINDOW_SIZE 256
#define RAW_BUFFER_SIZE 257  // Butuh 257 sampel untuk membuat 256 pasang waktu
#define STRIDE_32HZ 64       // Geser 64 sampel (2 detik di 32Hz) setelah inferensi

// Fungsi untuk memasukkan data dari sensor ke buffer
void ppg_add_sample_100hz(uint32_t raw_sample_100hz);
// Masukkan 2 sampel 64Hz sekaligus, fungsi ini akan merata-ratakannya menjadi 1 sampel 32Hz
void ppg_add_samples_64hz(uint32_t sample1_64hz, uint32_t sample2_64hz);

// Cek apakah buffer sudah terisi penuh (257 sampel) untuk inferensi pertama kali
bool ppg_is_buffer_ready();

// Fungsi gabungan: Z-Score dan Pengisian Tensor
void ppg_preprocess_and_feed(float* tflite_input_tensor);
void ppg_preprocess_and_feed_int8(int8_t* tflite_input_tensor, float scale, int32_t zero_point);

// Fungsi untuk menggeser buffer (Sliding Window) setelah inferensi selesai
void ppg_shift_buffer();

#ifdef __cplusplus
}
#endif

#endif