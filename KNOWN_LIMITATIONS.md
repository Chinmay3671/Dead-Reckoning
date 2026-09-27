# ReckonX — Known Technical Limitations & Architecture Boundary

## 1. AI Motion Error Correction Model (Browser vs. Mobile Runtime)

### Overview
In the upstream mobile architecture (`IDR_PRO`), a CNN-LSTM deep learning residual correction model (`ins_error_model_fused.tflite`, 134 KB) is used to predict residual velocity error corrections $[\delta v_N, \delta v_E]$ during GNSS outages. 

### Current Browser Status
1. **Native Mobile Dependency in Upstream:**
   - In `IDR_PRO`, model inference is executed via `react-native-fast-tflite`, which is a native C++/JSI iOS and Android runtime.
   - Standard `.tflite` binaries compiled with native TFLite kernels cannot be executed directly within standard browser DOM/Vite contexts without native bindings.
2. **Tensor Specifications & Preprocessing:**
   - **Input 1 (`serving_default_imu_window:0`):** `[1, 20, 6]` `float32` temporal sliding window of 20 normalized 6-DOF IMU samples ($a_x, a_y, a_z, g_x, g_y, g_z$).
   - **Input 2 (`serving_default_ins_state:0`):** `[1, 2]` `float32` current INS velocity state vector.
   - **Output (`StatefulPartitionedCall_1:0`):** `[1, 2]` `float32` normalized residual velocity corrections.
   - **Normalization Parameters:** Maintained in `src/services/fusion/AiMotionModel.ts` (`AI_NORMALIZATION_STATS`).
3. **Current Web Implementation (`src/services/fusion/AiMotionModel.ts`):**
   - The browser implementation encapsulates the sliding window buffering, state management, and normalization pipeline in TypeScript.
   - When the model is not loaded, it returns safe zero-residual vectors ($[0, 0]$), allowing the 15-state Error-State EKF (with ZUPT, NHC, and INS mechanization) to operate with full mathematical stability and zero drift spikes.
4. **Future Path for Client-Side Neural Inference:**
   - Convert the original Keras/PyTorch checkpoint to **TensorFlow.js Graph Model** format (`model.json` + binary shards) or **ONNX Web** runtime (`ort-web` / WebAssembly / WebGPU).
   - Drop the converted JSON/binaries into `public/models/` and load via `@tensorflow/tfjs` or `onnxruntime-web` in `AiMotionModel.ts`.

---

## 2. Hardware Sensor Sampling Constraints

1. **Browser Permission Workflows:**
   - Modern iOS Safari and Android Chrome require explicit user gesture permissions (`DeviceMotionEvent.requestPermission()`) before high-frequency sensor streaming is unlocked. ReckonX provides this via `/permissions`.
2. **Platform Sampling Rate Caps:**
   - On desktop browsers without physical IMU hardware, accelerometer and gyroscope streams report unavailable (`null`/`0`) in `live` mode per ReckonX's strict *No Fake Data* standard.
