#pragma once

struct ImuParams {
  int sampleFreq;
  float kp;
  float ki;
  float kiScale;
};

struct ImuState {
  bool fault;
  OrientationAngles orientation;
};

void updateOrientation(ImuParams* params, ImuState* state);
void initImu(ImuParams* params);
