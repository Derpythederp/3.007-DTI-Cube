float fract(float x);
float mix(float a, float b, float t);
float* hsv2rgb(float h, float s, float b, float* rgb);  // rgb is an array for rgb values normalized to 0.0 to 1.0, hsb are also normalized
float* rgb2hsv(float r, float g, float b, float* hsv);
