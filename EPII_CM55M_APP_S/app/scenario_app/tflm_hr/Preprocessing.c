#include "Preprocessing.h"
#include <math.h>

#define RAW_SAMPLE_PERIOD_MS  10.0f 

// ============================================================
// DC REMOVAL — IIR Highpass Filter
// Rumus: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
// alpha = tau / (tau + 1/fs) di mana tau = 1 / (2*pi*fc)
// fc = 0.5 Hz, fs = 32 Hz → alpha ≈ 0.9029
// Ini menghilangkan drift DC lambat tanpa mempengaruhi
// komponen pulsatil PPG (0.5–4 Hz)
// ============================================================
#define HPF_ALPHA   0.9029f 

// Buffer internal untuk menyimpan sinyal mentah (32Hz)
static float raw_buffer[RAW_BUFFER_SIZE];
static int buffer_index = 0;
static bool is_ready = false;

// Variabel state untuk interpolasi waktu nyata
static float time_100hz = 0.0f;
static float time_32hz = 0.0f;
static float prev_sample_100hz = 0.0f;
static bool is_first_sample = true;

// State IIR highpass filter
static float hpf_prev_input  = 0.0f;
static float hpf_prev_output = 0.0f;

static float apply_highpass_filter(float x_new) {
    float y_new = HPF_ALPHA * (hpf_prev_output + x_new - hpf_prev_input);
    hpf_prev_input  = x_new;
    hpf_prev_output = y_new;
    return y_new;
}

// ============================================================
// INTERNAL: Tulis satu sampel ke buffer setelah filter HPF
// ============================================================
static void write_to_buffer(float sample) {
    float filtered = apply_highpass_filter(sample);
    if (buffer_index < RAW_BUFFER_SIZE) {
        raw_buffer[buffer_index++] = filtered;
        if (buffer_index >= RAW_BUFFER_SIZE) {
            is_ready = true;
        }
    }
}

void ppg_add_sample_100hz(uint32_t raw_sample_100hz) {
     float curr = (float)raw_sample_100hz;

    if (is_first_sample) {
        prev_sample_100hz  = curr;
        time_100hz      = 0.0f;
        time_32hz       = 0.0f;
        is_first_sample = false;
        // Sampel pertama langsung tulis tanpa interpolasi
        write_to_buffer(curr);
        time_32hz += 31.25f;
        return;
    }

    // Majukan jam virtual 100 Hz (selalu, agar tidak diskontinuitas)
    time_100hz += RAW_SAMPLE_PERIOD_MS;  // +10ms per sampel

    // Hasilkan sampel 32 Hz selama interval ini masih ada
    while (time_32hz <= time_100hz && buffer_index < RAW_BUFFER_SIZE) {
        float t0           = time_100hz - RAW_SAMPLE_PERIOD_MS;
        float fraksi_waktu = (time_32hz - t0) / RAW_SAMPLE_PERIOD_MS;
        float interpolated = prev_sample_100hz +
                             (fraksi_waktu * (curr - prev_sample_100hz));

        write_to_buffer(interpolated);
        time_32hz += 31.25f;  // 1000ms / 32Hz = 31.25ms
    }

    prev_sample_100hz = curr;

    // Wrap-around untuk mencegah float overflow setelah ~100 detik
    if (time_100hz > 100000.0f) {
        time_100hz -= 100000.0f;
        time_32hz  -= 100000.0f;
    }
}

void ppg_add_samples_64hz(uint32_t sample1_64hz, uint32_t sample2_64hz) {
    // Abaikan jika buffer sudah penuh dan belum di-shift
    if (buffer_index >= RAW_BUFFER_SIZE) {
        return; 
    }

    // DOWNSAMPLING: Rata-ratakan 2 sampel 64Hz menjadi 1 sampel 32Hz
    // Cast dari uint32_t ke float
    float averaged_sample = ((float)sample1_64hz + (float)sample2_64hz) / 2.0f;

    // Masukkan ke buffer
    raw_buffer[buffer_index] = averaged_sample;
    buffer_index++;

    if (buffer_index >= RAW_BUFFER_SIZE) {
        is_ready = true;
    }
}

bool ppg_is_buffer_ready() {
    return is_ready;
}

void ppg_preprocess_and_feed(float* tflite_input_tensor) {
    if (!is_ready) return;

    // 1. Hitung Mean (Rata-rata)
    float sum = 0.0f;
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
        sum += raw_buffer[i];
    }
    float mean = sum / (float)RAW_BUFFER_SIZE;

    // 2. Hitung Standard Deviation (Simpangan Baku)
    float variance_sum = 0.0f;
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
        float diff = raw_buffer[i] - mean;
        variance_sum += (diff * diff);
    }
    // Gunakan sqrtf untuk efisiensi FPU 32-bit di mikrokontroler
    float std_dev = sqrtf(variance_sum / (float)RAW_BUFFER_SIZE);

    // Mencegah pembagian dengan nol jika data sensor konstan (datar)
    if (std_dev < 1e-5f) {
        std_dev = 1.0f;
    }

    // 3. Z-SCORE & FORMATTING TENSOR
    // Melakukan normalisasi dan langsung memetakannya ke array 1D TFLite [256 * 2]
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float norm_n       = (raw_buffer[i] - mean) / std_dev;
        float norm_n_plus1 = (raw_buffer[i + 1] - mean) / std_dev;
        
        tflite_input_tensor[(i * 2) + 0] = norm_n;
        tflite_input_tensor[(i * 2) + 1] = norm_n_plus1;
    }
}

void ppg_preprocess_and_feed_int8(int8_t* tflite_input_tensor, float scale, int32_t zero_point){
    if (!is_ready) return;

    // 1. Hitung Mean
    float sum = 0.0f;
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
        sum += raw_buffer[i];
    }
    float mean = sum / (float)RAW_BUFFER_SIZE;

    // 2. Hitung Standard Deviation
    float variance_sum = 0.0f;
    for (int i = 0; i < RAW_BUFFER_SIZE; i++) {
        float diff = raw_buffer[i] - mean;
        variance_sum += (diff * diff);
    }
    float std_dev = sqrtf(variance_sum / (float)RAW_BUFFER_SIZE);

    if (std_dev < 1e-5f) {
        std_dev = 1.0f;
    }

    // 3. Z-SCORE & MANUAL QUANTIZATION
    for (int i = 0; i < WINDOW_SIZE; i++) {
        // A. Hitung Z-Score (Float)
        float norm_n       = (raw_buffer[i] - mean) / std_dev;
        float norm_n_plus1 = (raw_buffer[i + 1] - mean) / std_dev;
        
        norm_n       = -norm_n;
        norm_n_plus1 = -norm_n_plus1;

        // B. Terapkan rumus kuantisasi Affine (Float -> INT32)
        int32_t q_n       = (int32_t)roundf(norm_n / scale) + zero_point;
        int32_t q_n_plus1 = (int32_t)roundf(norm_n_plus1 / scale) + zero_point;
        
        // C. Clamping: Pastikan nilai tidak bocor dari batas INT8 (-128 hingga 127)
        if (q_n < -128) q_n = -128;
        if (q_n > 127)  q_n = 127;
        
        if (q_n_plus1 < -128) q_n_plus1 = -128;
        if (q_n_plus1 > 127)  q_n_plus1 = 127;
        
        // D. Simpan ke Tensor INT8
        tflite_input_tensor[(i * 2) + 0] = (int8_t)q_n;
        tflite_input_tensor[(i * 2) + 1] = (int8_t)q_n_plus1;
    };
}

void ppg_shift_buffer() {
    // Geser sisa data ke kiri untuk Sliding Window (Overlap)
    // Sisakan data lama, buang STRIDE_32HZ data terlama
    int remaining_samples = RAW_BUFFER_SIZE - STRIDE_32HZ;
    
    for (int i = 0; i < remaining_samples; i++) {
        raw_buffer[i] = raw_buffer[i + STRIDE_32HZ];
    }
    
    // Kembalikan indeks agar bisa diisi data baru
    buffer_index = remaining_samples;
    is_ready = false; // Tunggu sampai buffer penuh 257 lagi
}

void ppg_reset_buffer(){
    buffer_index = 0;
    is_ready = false;
}