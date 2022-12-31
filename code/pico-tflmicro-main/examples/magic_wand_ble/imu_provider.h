#ifndef MAGIC_WAND_IMU_PROVIDER_H
#define MAGIC_WAND_IMU_PROVIDER_H
#define WINDOW_LENGTH 10
#define MAX_CURSOR_SPEED 10
#define SCREEN_LENGTH 3000
#define SCREEN_WIDTH 3000
#define ANGLEX_LIMIT 100.0
#define ANGLEY_LIMIT 100.0
// #define KEYBOARD_SWITCH_IO 28 // don't use pin4, it's for IMU on board 

#include "ICM20948.h"
#include <ICM42622.h>
#include "main_functions.h"

uint8_t DeviceWho = 0;

namespace {

constexpr int stroke_transmit_stride     = 2;
constexpr int stroke_transmit_max_length = 160;
constexpr int stroke_max_length = stroke_transmit_max_length * stroke_transmit_stride;
constexpr int stroke_points_byte_count =
  2 * sizeof(int8_t) * stroke_transmit_max_length;
constexpr int stroke_struct_byte_count =
  (2 * sizeof(int32_t)) + stroke_points_byte_count;
constexpr int moving_sample_count = 50;

static float current_velocity[3]        = { 0.0f, 0.0f, 0.0f };
static float current_gravity[3]         = { 0.0f, 0.0f, 0.0f };
static float current_gyroscope_drift[3] = { 0.0f, 0.0f, 0.0f };

static float vx_window[WINDOW_LENGTH] =  { 0.0f};
static float vy_window[WINDOW_LENGTH] =  { 0.0f};
static float vz_window[WINDOW_LENGTH] =  { 0.0f};

// Mouse parameters
int16_t mouse_position[2] = {0,0}; //cursor absolute position
int8_t mouse_movement[2] = {0,0}; //cursor relative movement
int16_t goal_position[2] = {0};
float angle_xz, angle_yz ;

static int32_t  stroke_length                                  = 0;
static uint8_t  stroke_struct_buffer[stroke_struct_byte_count] = {};
static int32_t *stroke_state = reinterpret_cast<int32_t *>(stroke_struct_buffer);
static int32_t *stroke_transmit_length =
  reinterpret_cast<int32_t *>(stroke_struct_buffer + sizeof(int32_t));
static int8_t *stroke_points =
  reinterpret_cast<int8_t *>(stroke_struct_buffer + (sizeof(int32_t) * 2));

// A buffer holding the last 600 sets of 3-channel values from the accelerometer.
constexpr int acceleration_data_length                        = 600 * 3; // the length of storage
float         acceleration_data[acceleration_data_length]     = {};
float         acceleration_data_tmp[acceleration_data_length] = {};

// The next free entry in the data array.
int   acceleration_data_index  = 0;
float acceleration_sample_rate = 0.0f;

// A buffer holding the last 600 sets of 3-channel values from the gyroscope.
constexpr int gyroscope_data_length                     = 600 * 3;
float         gyroscope_data[gyroscope_data_length]     = {};
float         gyroscope_data_tmp[gyroscope_data_length] = {};
float         orientation_data[gyroscope_data_length]   = {};
// The next free entry in the data array.
int   gyroscope_data_index  = 0;
float gyroscope_sample_rate = 0.0f;

bool gravity_ready_flag = false;
bool static_flag = true;
bool hid_sent_flag = false;
enum {
  eWaiting = 0,
  eDrawing = 1,
  eDone    = 2,
};

IMU_EN_SENSOR_TYPE enMotionSensorType;

TfLiteStatus SetupIMU(tflite::ErrorReporter *error_reporter) {
  uint8_t flag = 0;
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(4, GPIO_FUNC_I2C);
  gpio_set_function(5, GPIO_FUNC_I2C);
  gpio_pull_up(4);
  gpio_pull_up(5);
  sleep_ms(1000);
  uint8_t DeviceID = ICM42622::Icm42622CheckID();
  if (DeviceID == ICM42622_DEVICE_ID) {
    DeviceWho = ICM42622_DEVICE;
    flag      = ICM42622::Icm42622Init();
    if (flag == 0) {
      TF_LITE_REPORT_ERROR(error_reporter, "Failed to initialize IMU");
      return kTfLiteError;
    }
    acceleration_sample_rate = 100;
    gyroscope_sample_rate    = 100;
  }
  else {
    DeviceWho = ICM20948_DEVICE;
    ICM20948::imuInit(&enMotionSensorType);
    if (IMU_EN_SENSOR_TYPE_ICM20948 != enMotionSensorType) {
      TF_LITE_REPORT_ERROR(error_reporter, "Failed to initialize IMU");
      return kTfLiteError;
    }
    acceleration_sample_rate = 1125.0f / (8 + 1);
    gyroscope_sample_rate = 1125.0f / (8 + 1);
  }

  TF_LITE_REPORT_ERROR(error_reporter, "Magic starts!");
  return kTfLiteOk;
}
  
void ReadAccelerometerAndGyroscope(int *new_accelerometer_samples,
                                   int *new_gyroscope_samples) {
  // Keep track of whether we stored any new data
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;
  // Loop through new samples and add to buffer
    while (ICM42622::Icm42622DataReady()) {
      const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
      gyroscope_data_index += 3;
      float *current_gyroscope_data     = &gyroscope_data[gyroscope_index];
      float *current_gyroscope_data_tmp = &gyroscope_data_tmp[gyroscope_index];
      if (!ICM42622::Icm42622ReadGyro(&current_gyroscope_data_tmp[0],
                                      &current_gyroscope_data_tmp[1],
                                      &current_gyroscope_data_tmp[2])) {
        printf("Failed to read gyroscope data");
        break;
      }
      current_gyroscope_data[0] = -current_gyroscope_data_tmp[1];
      current_gyroscope_data[1] = -current_gyroscope_data_tmp[0];
      current_gyroscope_data[2] = current_gyroscope_data_tmp[2];
      *new_gyroscope_samples += 1;

      const int acceleration_index =
        (acceleration_data_index % acceleration_data_length);
      acceleration_data_index += 3;
      float *current_acceleration_data     = &acceleration_data[acceleration_index];
      float *current_acceleration_data_tmp = &acceleration_data_tmp[acceleration_index];
      // Read each sample, removing it from the device's FIFO buffer
      if (!ICM42622::Icm42622ReadAccel(&current_acceleration_data_tmp[0],
                                       &current_acceleration_data_tmp[1],
                                       &current_acceleration_data_tmp[2])) {
        printf("Failed to read acceleration data");
        break;
      }
      current_acceleration_data[0] = -current_acceleration_data_tmp[1];
      current_acceleration_data[1] = -current_acceleration_data_tmp[0];
      current_acceleration_data[2] = current_acceleration_data_tmp[2];
      *new_accelerometer_samples += 1; // It's a counter
      ICM42622::Icm42622DataReady();
      // printf("Current Accl data: %f %f %f \n", current_acceleration_data[0],current_acceleration_data[1],current_acceleration_data[2]);
      // printf("Current Gyro data: %f %f %f \n", current_gyroscope_data[0],current_gyroscope_data[1],current_gyroscope_data[2]);
    }
}

void ReadMPU6050(int *new_accelerometer_samples,int *new_gyroscope_samples,
                              float *acceleration1,float* gyro1) {
                                  char str[100];
  // sprintf(str,"__Acc1. _X = %d, _Y = %d, _Z = %d\n", acceleration1[0], acceleration1[1], acceleration1[2]);
  // uart_puts(uart0,str);
  // sprintf(str,"__Gyro1. _X = %d, _Y = %d, _Z = %d\n", gyro1[0], gyro1[1], gyro1[2]);
  // uart_puts(uart0,str);
  // Keep track of whether we stored any new data
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;
  // temporaily data stroage
  // Loop through new samples and add to buffer
      const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
      gyroscope_data_index += 3;
      float *current_gyroscope_data     = &gyroscope_data[gyroscope_index];
      const int acceleration_index =(acceleration_data_index % acceleration_data_length);
      acceleration_data_index += 3;
      float *current_acceleration_data     = &acceleration_data[acceleration_index];

      current_gyroscope_data[0] = -gyro1[1];
      current_gyroscope_data[1] = -gyro1[0];
      current_gyroscope_data[2] = gyro1[2];
      *new_gyroscope_samples += 1;

      current_acceleration_data[0] = -acceleration1[1];
      current_acceleration_data[1] = -acceleration1[0];
      current_acceleration_data[2] = acceleration1[2];
      *new_accelerometer_samples += 1;
      // char test_data[100];
      // sprintf(test_data,"acceleration1[0]=%.3f,gyro[0]=%.3f\n",acceleration1[0],gyro1[0]);
      // uart_puts(uart0,test_data);
      // char temp1[20];
      // sprintf(temp1, " %d  %d  %d \r\n", current_acceleration_data[0], current_acceleration_data[1], current_acceleration_data[2]);
      // uart_puts(uart0, temp1);
      // It's a counter
      // printf("Current Accl data: %f %f %f \n", current_acceleration_data[0],current_acceleration_data[1],current_acceleration_data[2]);
      // printf("Current Gyro data: %f %f %f \n", current_gyroscope_data[0],current_gyroscope_data[1],current_gyroscope_data[2]);
}


// created by pp    https://c.miaowlabs.com/A27.html
void ComplementaryFilter(float acc, float gyro, float dt,float &angle) 
{
    float a = 0.98;  
    angle = a * (angle + gyro * dt) + (1 - a) * (acc);  
    // return angle;  
}
// created by pp
void ReadAccelerometerAndGyroscope(float *new_accelerometer_samples,
                                   float *new_gyroscope_samples,
                                   float *angular) {
  // Keep track of whether we stored any new data
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;
  float elapsed_time = 1/gyroscope_sample_rate;
  float dt = 1/gyroscope_sample_rate;
  int count=0;
  if (DeviceWho == ICM42622_DEVICE) { // this is the chip on pico 4ML
    while (ICM42622::Icm42622DataReady()){
      count+=1;
      float gyro_tmp[3];
      float accel_tmp[3];
      if (!ICM42622::Icm42622ReadGyro(&gyro_tmp[0],
                                      &gyro_tmp[1],
                                      &gyro_tmp[2])) {
        printf("Failed to read gyroscope data");
        break;
      }

      // Read each sample, removing it from the device's FIFO buffer
      if (!ICM42622::Icm42622ReadAccel(&accel_tmp[0],
                                        &accel_tmp[1],
                                        &accel_tmp[2])) {
        printf("Failed to read acceleration data");
        break;
      }
      new_accelerometer_samples[0]=accel_tmp[0];
      new_accelerometer_samples[1]=accel_tmp[1];
      new_accelerometer_samples[2]=accel_tmp[2];
      // accel_tmp[0]/= 16384; //x
      // accel_tmp[1]/= 16384; //y
      // accel_tmp[2]/= 16384; //z
      float acc_xz, acc_yz;
      float gro_x,gro_y;
      acc_xz = atan2(accel_tmp[0],accel_tmp[2])*180/3.14;
      acc_yz = atan2(accel_tmp[1],accel_tmp[2])*180/3.14;
      gro_x= gyro_tmp[0] / 16.4;
      gro_y= gyro_tmp[1] / 16.4;
      float yAngle, xAngle;
      ComplementaryFilter(acc_xz,gro_x,dt,angular[0]);
      ComplementaryFilter(acc_yz,gro_y,dt,angular[1]);
    }
  }

}

// unused     -comment by pp
int ReadGyroscope() {
  // Keep track of whether we stored any new data
  int new_samples = 0;
  // Loop through new samples and add to buffer
  while (ICM42622::Icm42622DataReady()) {
    const int index = (gyroscope_data_index % gyroscope_data_length);
    gyroscope_data_index += 3;
    float *data     = &gyroscope_data[index];
    float *data_tmp = &gyroscope_data_tmp[index];

    // Read each sample, removing it from the device's FIFO buffer
    if (!ICM42622::Icm42622ReadGyro(&data_tmp[0], &data_tmp[1], &data_tmp[2])) {
      printf("Failed to read gyroscope data");
      break;
    }
    data[0] = -data_tmp[1];
    data[1] = data_tmp[0];
    data[2] = -data_tmp[2];
    new_samples += 1;
  }
  return new_samples;
}

// called inside updatestoke    -comment by pp
bool IsMoving(int samples_before) {
  // calculate if the Arduino is move using the mean squared difference
  // of the current and previous gyroscope data
  // Note: this is different from how we calulate isMoving in EstimateGyroscopeDrift()
  constexpr float moving_threshold = 10.0f;

  if ((gyroscope_data_index - samples_before) < moving_sample_count) {
    return false;
  }

  const int start_index =
    ((gyroscope_data_index
      + (gyroscope_data_length - (3 * (moving_sample_count + samples_before))))
     % gyroscope_data_length);

  float total = 0.0f;
  for (int i = 0; i < moving_sample_count; ++i) {
    const int index = ((start_index + (i * 3)) % gyroscope_data_length);
    float    *current_orientation = &orientation_data[index];
    const int previous_index = (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float *previous_orientation = &orientation_data[previous_index];
    const float  dx          = current_orientation[0] - previous_orientation[0];
    const float  dy          = current_orientation[1] - previous_orientation[1];
    const float  dz          = current_orientation[2] - previous_orientation[2];
    const float  mag_squared = (dx * dx) + (dy * dy) + (dz * dz);
    total += mag_squared;
  }
  const bool is_moving = (total > moving_threshold);
  return is_moving;
}


float VectorMagnitude(const float *vec) {
  const float x = vec[0];
  const float y = vec[1];
  const float z = vec[2];
  return sqrtf((x * x) + (y * y) + (z * z));
}

void NormalizeVector(const float *in_vec, float *out_vec) {
  const float magnitude = VectorMagnitude(in_vec);
  const float x         = in_vec[0];
  const float y         = in_vec[1];
  const float z         = in_vec[2];
  out_vec[0]            = x / magnitude;
  out_vec[1]            = y / magnitude;
  out_vec[2]            = z / magnitude;
}

float DotProduct(const float *a, const float *b) {
  return (a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

// use acceleration data to calculate gravity[3]    -comment by pp
void EstimateGravityDirection(float *gravity) {
  int samples_to_average = 100;
  if (samples_to_average >= acceleration_data_index) {
    samples_to_average = acceleration_data_index;
  }
  // waiting the value to be stablized
  if (gravity_ready_flag == false && acceleration_data_index > 300){ 
    gravity_ready_flag = true;
  }
  const int start_index =
    ((acceleration_data_index
      + (acceleration_data_length - (3 * (samples_to_average + 1))))
     % acceleration_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int    index = ((start_index + (i * 3)) % acceleration_data_length);
    const float *entry = &acceleration_data[index];
    const float  x     = entry[0];
    const float  y     = entry[1];
    const float  z     = entry[2];
    x_total += x;
    y_total += y;
    z_total += z;
  }
  gravity[0] = x_total / samples_to_average;
  gravity[1] = y_total / samples_to_average;
  gravity[2] = z_total / samples_to_average;

  angle_xz = atan2(gravity[0],gravity[2])*180/3.14; 
  angle_yz = atan2(gravity[1],gravity[2])*180/3.14;
  // char temp1[20];
  // sprintf(temp1, "\n%f\t%f", angle_xz, angle_yz);
  // uart_puts(uart0, temp1);
}

// velocity filter(unused now)     -comment by pp
float sliding_window_filtering(float *window, float new_value){
  float sum = 0.0f;
  for (int i = 0; i < WINDOW_LENGTH-1; i++){
    window[i] = window[i+1];
    sum += window[i];
  }
  window[WINDOW_LENGTH-1] = new_value;
  return (sum + new_value)/10.0;
}

// output: relative movement to goal position on the screen
// void calculate_movement(){
//   int16_t goal_position[2] = {0};
//   if (angle_xz < -ANGLEX_LIMIT) angle_xz = -ANGLEX_LIMIT;
//   if (angle_xz > ANGLEX_LIMIT) angle_xz = ANGLEX_LIMIT;
//   if (angle_yz < -ANGLEY_LIMIT) angle_yz = -ANGLEY_LIMIT;
//   if (angle_yz > ANGLEY_LIMIT) angle_yz = ANGLEY_LIMIT;
//   goal_position[0] = int16_t(angle_xz * SCREEN_LENGTH / (2*ANGLEX_LIMIT));
//   goal_position[1] = int16_t(angle_yz * SCREEN_WIDTH / (2*ANGLEY_LIMIT));
//   mouse_movement[0] = int8_t(goal_position[0] - mouse_position[0]);
//   mouse_movement[1] = int8_t(goal_position[1] - mouse_position[1]);
//   if (mouse_movement[0] > MAX_CURSOR_SPEED) mouse_movement[0] = MAX_CURSOR_SPEED;
//   if (mouse_movement[0] < -MAX_CURSOR_SPEED) mouse_movement[0] = -MAX_CURSOR_SPEED;
//   if (mouse_movement[1] > MAX_CURSOR_SPEED) mouse_movement[1] = MAX_CURSOR_SPEED;
//   if (mouse_movement[1] < -MAX_CURSOR_SPEED) mouse_movement[1] = -MAX_CURSOR_SPEED;
//   if (hid_sent_flag == true){
//   mouse_position[0] = mouse_position[0] + int16_t(mouse_movement[0]);
//   mouse_position[1] = mouse_position[1] + int16_t(mouse_movement[1]);  
//   }
//   char temp1[20];
//   sprintf(temp1, "\n%d  %d  %d \r\n", mouse_position[0], goal_position[0], mouse_movement[0]);
//   // uart_puts(uart0, temp1);
// }

// let |velocity|<=1
void velocity_normalization(){
if (current_velocity[0] > 1){
  current_velocity[0] = 1;
}
if (current_velocity[0] < -1){
  current_velocity[0] = -1;
}
if (current_velocity[1] > 1){
  current_velocity[1] = 1;
}
if (current_velocity[1] < -1){
  current_velocity[1] = -1;
}
if (current_velocity[2] > 1){
  current_velocity[2] = 1;
}
if (current_velocity[2] < -1){
  current_velocity[2] = -1;
}
}

void UpdateVelocity(int new_samples, float *gravity) { // use gyro to judge the movement
  const float gravity_x = gravity[0];
  const float gravity_y = gravity[1];
  const float gravity_z = gravity[2];

  const int start_index =
    ((acceleration_data_index + (acceleration_data_length - (3 * (new_samples + 1))))
     % acceleration_data_length);

  const float friction_fudge = 0.98f; 

  for (int i = 0; i < new_samples; ++i) { // here the new samples is from accerlerometer!
    const int    index = ((start_index + (i * 3)) % acceleration_data_length);
    const float *entry = &acceleration_data[index];
    const float  ax    = entry[0];
    const float  ay    = entry[1];
    const float  az    = entry[2];

    // Try to remove gravity from the raw acceleration values.
    const float ax_minus_gravity = (int16_t((ax - gravity_x)*10))*0.1; //keep 2 digits
    const float ay_minus_gravity = (int16_t((ay - gravity_y)*10))*0.1;
    const float az_minus_gravity = (int16_t((az - gravity_z)*10))*0.1;
    // Update velocity based on the normalized acceleration.
    if (ax_minus_gravity * ax_minus_gravity + ay_minus_gravity * ay_minus_gravity + az_minus_gravity * az_minus_gravity< 0.01f){ //consider adding gyro data for movement judge
      // when it's static, velocity goes to zero
      current_velocity[0] = 0;
      current_velocity[1] = 0;
      current_velocity[2] = 0;  
      static_flag = true;
    }
    else{ // the board has movement
      static_flag = false;
      if (gravity_ready_flag == true){ // when collect enough gravity data
      current_velocity[0] += ax_minus_gravity;
      current_velocity[1] += ay_minus_gravity;
      current_velocity[2] += az_minus_gravity;
      }  
      velocity_normalization();
    }
    // current_velocity[0] = sliding_window_filtering(vx_window, current_velocity[0]);
    // current_velocity[1] = sliding_window_filtering(vy_window, current_velocity[1]);
    // current_velocity[2] = sliding_window_filtering(vz_window, current_velocity[2]);
    // Dampen the velocity slightly with a fudge factor to stop it exploding.
    // current_velocity[0] *= friction_fudge;
    // current_velocity[1] *= friction_fudge;
    // current_velocity[2] *= friction_fudge;
  }
}

void UpdatePosition(float *current_position){
  float last_position[3] = {0.0f, 0.0f, 0.0f};
  current_position[0] += current_velocity[0]; // problem: the velocity value explode
  current_position[1] += current_velocity[1];
  current_position[2] += current_velocity[2];
   if (static_flag){ 
      // when it's static, velocity goes to zero
      current_position[0] = 0;
      current_position[1] = 0;
      current_position[2] = 0;  
    }
}

void EstimateGyroscopeDrift(float *drift) {
  // Estimate and update the drift of the gyroscope when the Ardiuno is not moving
  const bool isMoving = VectorMagnitude(current_velocity) > 0.1f;
  if (isMoving) {
    return;
  }

  int samples_to_average = 20;
  if (samples_to_average >= gyroscope_data_index) {
    samples_to_average = gyroscope_data_index;
  }

  const int start_index =
    ((gyroscope_data_index + (gyroscope_data_length - (3 * (samples_to_average + 1))))
     % gyroscope_data_length);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int    index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float *entry = &gyroscope_data[index];
    const float  x     = entry[0];
    const float  y     = entry[1];
    const float  z     = entry[2];
    x_total += x;
    y_total += y;
    z_total += z;
  }
  drift[0] = x_total / samples_to_average;
  drift[1] = y_total / samples_to_average;
  drift[2] = z_total / samples_to_average;
}

void UpdateOrientation(int new_samples, float *gravity, float *drift) {
  // update the current orientation by integrating the angular velocity over time
  const float drift_x = drift[0];
  const float drift_y = drift[1];
  const float drift_z = drift[2];

  const int start_index =
    ((gyroscope_data_index + (gyroscope_data_length - (3 * new_samples)))
     % gyroscope_data_length);

  // The gyroscope values are in degrees-per-second, so to approximate
  // degrees in the integrated orientation, we need to divide each value
  // by the number of samples each second.
  const float recip_sample_rate = 1.0f / gyroscope_sample_rate;

  for (int i = 0; i < new_samples; ++i) {
    const int    index = ((start_index + (i * 3)) % gyroscope_data_length);
    const float *entry = &gyroscope_data[index];
    const float  dx    = entry[0];
    const float  dy    = entry[1];
    const float  dz    = entry[2];

    // Try to remove sensor errors from the raw gyroscope values.
    const float dx_minus_drift = dx - drift_x;
    const float dy_minus_drift = dy - drift_y;
    const float dz_minus_drift = dz - drift_z;

    // Convert from degrees-per-second to appropriate units for this
    // time interval.
    const float dx_normalized = dx_minus_drift * recip_sample_rate;
    const float dy_normalized = dy_minus_drift * recip_sample_rate;
    const float dz_normalized = dz_minus_drift * recip_sample_rate;

    // Update orientation based on the gyroscope data.
    float    *current_orientation = &orientation_data[index];
    const int previous_index =
      (index + (gyroscope_data_length - 3)) % gyroscope_data_length;
    const float *previous_orientation = &orientation_data[previous_index];
    current_orientation[0]            = previous_orientation[0] + dx_normalized;
    current_orientation[1]            = previous_orientation[1] + dy_normalized;
    current_orientation[2]            = previous_orientation[2] + dz_normalized;
  }

}

void UpdateStroke(int new_samples, bool *done_just_triggered) {
  // Take the angular values and project them into an XY plane

  constexpr int   minimum_stroke_length = moving_sample_count + 10;
  constexpr float minimum_stroke_size   = 0.0f;

  *done_just_triggered = false;

  // iterate through the new samples
  for (int i = 0; i < new_samples; ++i) {
    const int     current_head = (new_samples - (i + 1));
    const bool    is_moving    = IsMoving(current_head);
    const int32_t old_state    = *stroke_state;

    // determine if there is a break between gestures
    if ((old_state == eWaiting) || (old_state == eDone)) {
      if (is_moving) {
        stroke_length = moving_sample_count;
        *stroke_state = eDrawing;
      }
    }
    else if (old_state == eDrawing) {
      if (!is_moving) {
        if (stroke_length > minimum_stroke_length) {
          *stroke_state = eDone;
        }
        else {
          stroke_length = 0;
          *stroke_state = eWaiting;
        }
      }
    }

    // if the stroke is too small we skip to the next iteration
    const bool is_waiting = (*stroke_state == eWaiting);
    if (is_waiting) {
      continue;
    }

    stroke_length += 1;
    if (stroke_length > stroke_max_length) {
      stroke_length = stroke_max_length;
    }

    // Only recalculate the full stroke if it's needed.
    const bool draw_last_point = ((i == (new_samples - 1)) && (*stroke_state == eDrawing));
    *done_just_triggered = ((old_state != eDone) && (*stroke_state == eDone));
    if (!(*done_just_triggered || draw_last_point)) {
      continue;
    }

    const int start_index =
      ((gyroscope_data_index + (gyroscope_data_length - (3 * (stroke_length + current_head)))) % gyroscope_data_length);

    // accumulate the x, y, and z orintation data
    float x_total = 0.0f;
    float y_total = 0.0f;
    float z_total = 0.0f;
    for (int j = 0; j < stroke_length; ++j) {
      const int    index = ((start_index + (j * 3)) % gyroscope_data_length);
      const float *entry = &orientation_data[index];
      x_total += entry[0];
      y_total += entry[1];
      z_total += entry[2];
    }

    const float     x_mean = x_total / stroke_length;
    const float     y_mean = y_total / stroke_length;
    const float     z_mean = z_total / stroke_length;
    constexpr float range  = 90.0f;

    // Account for the roll orientation of the Arduino
    const float gy   = current_gravity[1];
    const float gz   = current_gravity[2];
    float       gmag = sqrtf((gy * gy) + (gz * gz));// in y,z plane!
    if (gmag < 0.0001f) {
      gmag = 0.0001f;
    }
    const float ngy = gy / gmag;
    const float ngz = gz / gmag;

    const float xaxisz = -ngz;
    const float xaxisy = -ngy;

    const float yaxisz = -ngy;
    const float yaxisy = ngz;

    *stroke_transmit_length = stroke_length / stroke_transmit_stride;

    // project the angular orientation into the 2d X/Y plane
    float x_min;
    float y_min;
    float x_max;
    float y_max;
    for (int j = 0; j < *stroke_transmit_length; ++j) {
      const int orientation_index =
        ((start_index + ((j * stroke_transmit_stride) * 3)) % gyroscope_data_length);
      const float *orientation_entry = &orientation_data[orientation_index];

      const float orientation_x = orientation_entry[0];
      const float orientation_y = orientation_entry[1];
      const float orientation_z = orientation_entry[2];

      const float nx = (orientation_x - x_mean) / range;
      const float ny = (orientation_y - y_mean) / range;
      const float nz = (orientation_z - z_mean) / range;

      const float x_axis = (xaxisz * nz) + (xaxisy * ny);
      const float y_axis = (yaxisz * nz) + (yaxisy * ny);

      const int stroke_index = j * 2;
      int8_t   *stroke_entry = &stroke_points[stroke_index];

      // cap the x/y values at -128 and 127 (int8)
      int32_t unchecked_x = static_cast<int32_t>(roundf(x_axis * 128.0f));
      int8_t  stored_x;
      if (unchecked_x > 127) {
        stored_x = 127;
      }
      else if (unchecked_x < -128) {
        stored_x = -128;
      }
      else {
        stored_x = unchecked_x;
      }
      stroke_entry[0] = stored_x;

      int32_t unchecked_y = static_cast<int32_t>(roundf(y_axis * 128.0f));
      int8_t  stored_y;
      if (unchecked_y > 127) {
        stored_y = 127;
      }
      else if (unchecked_y < -128) {
        stored_y = -128;
      }
      else {
        stored_y = unchecked_y;
      }
      stroke_entry[1] = stored_y;

      const bool is_first = (j == 0);
      if (is_first || (x_axis < x_min)) {
        x_min = x_axis;
      }
      if (is_first || (y_axis < y_min)) {
        y_min = y_axis;
      }
      if (is_first || (x_axis > x_max)) {
        x_max = x_axis;
      }
      if (is_first || (y_axis > y_max)) {
        y_max = y_axis;
      }
    }

    // If the stroke is too small, cancel it.
    if (*done_just_triggered) {
      const float x_range = (x_max - x_min);
      const float y_range = (y_max - y_min);
      if ((x_range < minimum_stroke_size) && (y_range < minimum_stroke_size)) {
        *done_just_triggered    = false;
        *stroke_state           = eWaiting;
        *stroke_transmit_length = 0;
        stroke_length           = 0;
      }
    }
  }
}
}  // namespace

////////mouse related below
///////////////////////////////////////////////////////
/////////////////////////////////////////// created by pp
void ReadAccelAndGyro(int *new_gyroscope_samples,
                      float* gyro_data, 
                      int & gyro_data_i, 
                      const int gyro_data_l,
                      int *new_accelerometer_samples,
                      float* accel_data,
                      int & accel_data_i, 
                      const int accel_data_l) {
  // Keep track of whether we stored any new data
  *new_accelerometer_samples = 0;
  *new_gyroscope_samples     = 0;
  // Loop through new samples and add to buffer
    while (ICM42622::Icm42622DataReady()) {
      const int gyroscope_index = (gyro_data_i % gyro_data_l);
      gyro_data_i += 3;
      float *current_gyroscope_data     = &gyro_data[gyroscope_index];
      // float *current_gyroscope_data_tmp = &gyroscope_data_tmp[gyroscope_index];
      float gyro_tmp[3];
      if (!ICM42622::Icm42622ReadGyro(&gyro_tmp[0],
                                      &gyro_tmp[1],
                                      &gyro_tmp[2])) {
        printf("Failed to read gyroscope data");
        break;
      }
      current_gyroscope_data[0] = -gyro_tmp[1];
      current_gyroscope_data[1] = -gyro_tmp[0];
      current_gyroscope_data[2] = gyro_tmp[2];
      *new_gyroscope_samples += 1;

      const int acceleration_index = (accel_data_i % accel_data_l);
      accel_data_i += 3;
      float *current_acceleration_data     = &accel_data[acceleration_index];

      float accel_tmp[3];
      // Read each sample, removing it from the device's FIFO buffer
      if (!ICM42622::Icm42622ReadAccel(&accel_tmp[0],
                                       &accel_tmp[1],
                                       &accel_tmp[2])) {
        printf("Failed to read acceleration data");
        break;
      }
      current_acceleration_data[0] = -accel_tmp[1];
      current_acceleration_data[1] = -accel_tmp[0];
      current_acceleration_data[2] = accel_tmp[2];
      *new_accelerometer_samples += 1; // It's a counter
      ICM42622::Icm42622DataReady();
      // printf("Current Accl data: %f %f %f \n", current_acceleration_data[0],current_acceleration_data[1],current_acceleration_data[2]);
      // printf("Current Gyro data: %f %f %f \n", current_gyroscope_data[0],current_gyroscope_data[1],current_gyroscope_data[2]);
    }
}

// use acceleration data to calculate gravity[3]    -comment by pp
void EstimateGrvtyDir(float* accel_data ,int accel_data_i, int accel_data_l ,
                          float *gravity,bool & is_gravity_ready, float & __angle_xz, float & __angle_yz) {
  int samples_to_average = 100;   // slide window size
  if (samples_to_average >= accel_data_i) {
    samples_to_average = accel_data_i;
  }
  // waiting the value to be stablized
  if (is_gravity_ready == false && accel_data_i > 300){ 
    is_gravity_ready = true;
  }
  const int start_index =
    ((accel_data_i + (accel_data_l - (3 * (samples_to_average + 1))))
     % accel_data_l);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int    index = ((start_index + (i * 3)) % accel_data_l);
    const float *entry = &accel_data[index];
    const float  x     = entry[0];
    const float  y     = entry[1];
    const float  z     = entry[2];
    x_total += x;
    y_total += y;
    z_total += z;
  }
  gravity[0] = x_total / samples_to_average;
  gravity[1] = y_total / samples_to_average;
  gravity[2] = z_total / samples_to_average;

  __angle_xz = atan2(gravity[0],gravity[2])*180/3.14; 
  __angle_yz = atan2(gravity[1],gravity[2])*180/3.14;
  // char temp1[20];
  // sprintf(temp1, "\n%f\t%f", angle_xz, angle_yz);
  // uart_puts(uart0, temp1);
}

// output: relative movement to goal position on the screen
void calculate_movement(float _angle_xz,float _angle_yz,int8_t* _mouse_movement,bool hid_sent_flag){
  
  if (_angle_xz < -ANGLEX_LIMIT) _angle_xz = -ANGLEX_LIMIT;
  if (_angle_xz > ANGLEX_LIMIT) _angle_xz = ANGLEX_LIMIT;
  if (_angle_yz < -ANGLEY_LIMIT) _angle_yz = -ANGLEY_LIMIT;
  if (_angle_yz > ANGLEY_LIMIT) _angle_yz = ANGLEY_LIMIT;
  goal_position[0] = int16_t(_angle_xz * SCREEN_LENGTH / (2*ANGLEX_LIMIT));
  goal_position[1] = int16_t(_angle_yz * SCREEN_WIDTH / (2*ANGLEY_LIMIT));
if (gpio_get(KEYBOARD_SWITCH_IO) == false){
  _mouse_movement[0] = int8_t(goal_position[0] - mouse_position[0]);
  _mouse_movement[1] = int8_t(goal_position[1] - mouse_position[1]);
}
  if (_mouse_movement[0] > MAX_CURSOR_SPEED) _mouse_movement[0] = MAX_CURSOR_SPEED;
  if (_mouse_movement[0] < -MAX_CURSOR_SPEED) _mouse_movement[0] = -MAX_CURSOR_SPEED;
  if (_mouse_movement[1] > MAX_CURSOR_SPEED) _mouse_movement[1] = MAX_CURSOR_SPEED;
  if (_mouse_movement[1] < -MAX_CURSOR_SPEED) _mouse_movement[1] = -MAX_CURSOR_SPEED;
  if (hid_sent_flag == true){
  mouse_position[0] = mouse_position[0] + int16_t(_mouse_movement[0]);
  mouse_position[1] = mouse_position[1] + int16_t(_mouse_movement[1]);  
  }
  // char temp1[20];
  // sprintf(temp1, "\n%d  %d  %d \r\n", mouse_position[0], goal_position[0], _mouse_movement[0]);
  // uart_puts(uart0, temp1);
}

void initialize_cursor_position(float _angle_xz,float _angle_yz)
{
  mouse_position[0] = int16_t(_angle_xz * SCREEN_LENGTH / (2*ANGLEX_LIMIT));
  mouse_position[1] = int16_t(_angle_yz * SCREEN_LENGTH / (2*ANGLEX_LIMIT));
}

#endif  // MAGIC_WAND_IMU_PROVIDER_H
