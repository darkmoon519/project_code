/* Copyright 2020 The TensorFlow Authors. All Rights Reserved.
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "hardware/gpio.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>

#include "imu_provider.h"
#include "magic_wand_model_data.h"
#include "rasterize_stroke.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "tusb_hid/hid_src.h"
#include "pico/stdlib.h"
#include "main_functions.h"

#define SCREEN 1

#if SCREEN

#include "LCD_st7735.h"

#endif

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define UART1_TX_PIN 4
#define UART1_RX_PIN 5
#define ADDR_PIN1 26
#define ADDR_PIN2 27
#define CLICK_THRES 1.0
namespace {
bool     linked     = false;
bool     first      = true;
uint16_t send_index = 0;

// Constants for image rasterization
constexpr int raster_width      = 32;
constexpr int raster_height     = 32;
constexpr int raster_channels   = 3;
constexpr int raster_byte_count = raster_height * raster_width * raster_channels;
int8_t        raster_buffer[raster_byte_count];

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 7 * 1024 + 1888;
uint8_t       tensor_arena[kTensorArenaSize];

tflite::ErrorReporter    *error_reporter = nullptr;
const tflite::Model      *model          = nullptr;
tflite::MicroInterpreter *interpreter    = nullptr;

// -------------------------------------------------------------------------------- //
// UPDATE THESE VARIABLES TO MATCH THE NUMBER AND LIST OF GESTURES IN YOUR DATASET  //
// -------------------------------------------------------------------------------- //
constexpr int label_count       = 10;
const char *labels[label_count] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };
}  // namespace

#ifndef DO_NOT_OUTPUT_TO_UART
// RX interrupt handler
uint8_t command[32]   = { 0 };
bool    start_flag    = false;
int     receive_index = 0;
uint8_t previous_ch   = 0;

void on_uart_rx() {
  uint8_t cameraCommand = 0;
  while (uart_is_readable(UART_ID)) {
    cameraCommand = uart_getc(UART_ID);
    //    //printf("%c \n", cameraCommand);
    if (start_flag) {
      command[receive_index++] = cameraCommand;
    }
    if (cameraCommand == 0xf4 && previous_ch == 0xf5) {
      start_flag = true;
    }
    else if (cameraCommand == 0x0a && previous_ch == 0x0d) {
      start_flag = false;
      // add terminator
      command[receive_index - 2] = '\0';

      receive_index = 0;
      if (strcmp("IND=BLECONNECTED", (const char *)command) == 0) {
        linked = true;
      }
      else if (strcmp("IND=BLEDISCONNECTED", (const char *)command) == 0) {
        linked = false;
      }
      //printf("%s\n", command);
    }
    previous_ch = cameraCommand;
  }
}

void setup_uart() {
  // Set up our UART with the required speed.
  uint baud = uart_init(UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);
  // Set up a RX interrupt
  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);
}

#else
void setup_uart() {ST7735_FillScreen(ST7735_RED);}
#endif

//static int addr = 0x69; // when AD0 is HIGH
// #ifdef i2c_default
void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    uint8_t addr1 = 0x68;
    uint8_t addr2 = 0x69;
    i2c_write_blocking(i2c1, addr1, buf, 2, false);
    i2c_write_blocking(i2c1, addr2, buf, 2, false);
}
void mpu6050_read_raw(uint8_t addr, int16_t accel[3], int16_t gyro[3]) { // add int16_t *temp for temperature if needs
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 59 (0x3B) for 6 bytes, x,y,z each with two bits
    // default +- 2g, 16384 b/g
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c1, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 67 (0x43) for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c1, addr, &val, 1, true);
    i2c_read_blocking(i2c1, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // // Now temperature from reg 0x41 for 2 bytes
    // // The register is auto incrementing on each read
    // val = 0x41;
    // i2c_write_blocking(i2c1, addr, &val, 1, true);
    // i2c_read_blocking(i2c1, addr, buffer, 2, false);  // False - finished with bus

    // *temp = buffer[0] << 8 | buffer[1];
}
// #endif
void mpu6050_read_data(uint8_t addr, float _accel[3], float _gyro[3]) { // add int16_t *temp for temperature if needs
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 59 (0x3B) for 6 bytes, x,y,z each with two bits
    // default +- 2g, 16384 b/g
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c1, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c1, addr, buffer, 6, false);

    int16_t accel[3];
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        _accel[i]=(float)accel[i]*2.0 / 32768.0;
    }

    int16_t gyro[3];
    // Now gyro data from reg 67 (0x43) for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c1, addr, &val, 1, true);
    i2c_read_blocking(i2c1, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        _gyro[i]=(float)gyro[i]*250.0 / 32768.0;
    }
}
// #endif

void ReadICM42622(float acc[3], float gyro[3]){
  ICM42622::Icm42622ReadAccel(&acc[0],
                              &acc[1],
                              &acc[2]);
  ICM42622::Icm42622ReadGyro(&gyro[0],
                             &gyro[1],
                             &gyro[2]);                            
}

void imu_print() {
    int16_t acceleration1[3], gyro1[3], temp1;
    int16_t acceleration2[3], gyro2[3], temp2;
    mpu6050_read_raw(0x68, acceleration1, gyro1);// &temp1
    char str[100];
    sprintf(str,"Acc1. X = %d, Y = %d, Z = %d\n", acceleration1[0], acceleration1[1], acceleration1[2]);
    uart_puts(uart0,str);
    sprintf(str,"Gyro1. X = %d, Y = %d, Z = %d\n", gyro1[0], gyro1[1], gyro1[2]);
    uart_puts(uart0,str);
    
    // mpu6050_read_raw(0x69, acceleration2, gyro2);// &temp2
    // These are the raw numbers from the chip, so will need tweaking to be really useful.
    printf("Acc1. X = %d, Y = %d, Z = %d\n", acceleration1[0], acceleration1[1], acceleration1[2]);
    // printf("Acc2. X = %d, Y = %d, Z = %d\n", acceleration2[0], acceleration2[1], acceleration2[2]);
    //printf("Gyro1. X = %d, Y = %d, Z = %d\n", gyro1[0], gyro1[1], gyro1[2]);
    //printf("Gyro2. X = %d, Y = %d, Z = %d\n", gyro2[0], gyro2[1], gyro2[2]);
    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    ////printf("Temp. = %f\n", (temp1 / 340.0) + 36.53);
}

// #define 
void setup() {
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  gpio_put(25, !gpio_get(25));
  
  // keyboard switch button
  gpio_init(KEYBOARD_SWITCH_IO);
  gpio_set_dir(KEYBOARD_SWITCH_IO, GPIO_IN);
  gpio_pull_up(KEYBOARD_SWITCH_IO);
#if SCREEN
  ST7735_Init();
  ST7735_DrawImage(0, 0, 80, 160, arducam_logo);
#endif

  board_init();
  tusb_init();
  // setup_uart();// Start serial
  // stdio_usb_init(); //can't use under tinyusb mode
  
/*Set up UART0 for test debugging*/
  uart_init(uart0, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_puts(uart0, " UART0 Setup!\n");
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  static tflite::MicroErrorReporter micro_error_reporter;  // NOLINT
  error_reporter = &micro_error_reporter;

  // Start IMU
  SetupIMU(error_reporter);

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroMutableOpResolver<4> micro_op_resolver;  // NOLINT
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddMean();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
    model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  interpreter->AllocateTensors();

  // Set model input settings
  TfLiteTensor *model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1)
      || (model_input->dims->data[1] != raster_height)
      || (model_input->dims->data[2] != raster_width)
      || (model_input->dims->data[3] != raster_channels)
      || (model_input->type != kTfLiteInt8)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Bad input tensor parameters in model");
    return;
  }

  // Set model output settings
  TfLiteTensor *model_output = interpreter->output(0);
  if ((model_output->dims->size != 2) || (model_output->dims->data[0] != 1)
      || (model_output->dims->data[1] != label_count)
      || (model_output->type != kTfLiteInt8)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Bad output tensor parameters in model");
    return;
  }
#if SCREEN
  ST7735_FillScreen(ST7735_GREEN);
  // char str1[10], str2[10], str3[10];
  // sprintf(str1, "x:");
  // sprintf(str2, "y:");
  // sprintf(str3, "z:");
  // ST7735_WriteString(5, 45, str1, Font_11x18, ST7735_BLACK, ST7735_GREEN);
  // ST7735_DrawImage(0, 0, 80, 40, (uint8_t *)IMU_ICM20948);
  // ST7735_WriteString(5, 45, "Magic", Font_11x18, ST7735_BLACK, ST7735_GREEN);
  // ST7735_WriteString(30, 70, "Wand", Font_11x18, ST7735_BLACK, ST7735_GREEN);
#endif
  gpio_put(25, !gpio_get(25));

/* initialize the MPU6050 */
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C)); // use GP2 (SDA) and GP3 (SCL)
  mpu6050_reset();
  // set up the address control pins
  gpio_set_dir(ADDR_PIN1, GPIO_OUT);
  gpio_set_dir(ADDR_PIN2, GPIO_OUT);
  gpio_put(ADDR_PIN1, 0);
  gpio_put(ADDR_PIN2, 1);
  // printf("Finish initialize MPU6050! Reading raw data from registers...\n");
  // estimate the gravity in ICM42622
  EstimateGravityDirection(current_gravity);
}

// void mouse_rel_movement(){ // relative movement of mouse
//   int accelerometer_samples_read;
//   int gyroscope_samples_read;
//   ReadAccelerometerAndGyroscope(&accelerometer_samples_read, &gyroscope_samples_read);  
//    if (accelerometer_samples_read > 0) {  
//     EstimateGravityDirection(current_gravity); 
//     UpdateVelocity(accelerometer_samples_read, current_gravity);
//   }
//   tud_task();
//   hid_task();
// }

void mouse_abs_position(){ // input is the cursor position
  // int accelerometer_samples_read;
  // int gyroscope_samples_read;
  
  // ReadAccelerometerAndGyroscope(&accelerometer_samples_read, &gyroscope_samples_read);  
  // if (accelerometer_samples_read > 0) {  
  //   EstimateGravityDirection(current_gravity); 
  //   UpdateVelocity(accelerometer_samples_read, current_gravity);
  // }
  // calculate_movement();
  // tud_task();
  // get_data(mouse_movement);
  // hid_task(hid_sent_flag); // make sure hid run the movement command
  static int gyroscope_samples_read;     // flag
  static int   i_mouse_gyro  = 0;
  constexpr int l_mouse_gyroscope = 600 * 3;
  static float mouse_gyro[l_mouse_gyroscope];
  
  static int accelerometer_samples_read;     // flag
  static int   i_mouse_accel  = 0;
  constexpr int l_mouse_accelerometer = 600 * 3;
  static float mouse_accel[l_mouse_accelerometer];

  static float gravity_now[3];
  static bool is_gravity_ready_flag=false;
  static float mouse_angle_xz, mouse_angle_yz;
  static int8_t  mouse_movement[2]={0};

  ReadAccelAndGyro(&gyroscope_samples_read,
                   mouse_gyro,
                   i_mouse_gyro,
                   l_mouse_gyroscope,
                   &accelerometer_samples_read,
                   mouse_accel,
                   i_mouse_accel,
                   l_mouse_accelerometer);  
   if (accelerometer_samples_read > 0) {  
      EstimateGrvtyDir( mouse_accel,
                        i_mouse_accel,
                        l_mouse_accelerometer,
                        gravity_now,
                        is_gravity_ready_flag,
                        mouse_angle_xz,
                        mouse_angle_yz); 
    
    // UpdateVelocity(accelerometer_samples_read, gravity_now);
  }
  calculate_movement(mouse_angle_xz, mouse_angle_yz, mouse_movement, hid_sent_flag);
  // tud_task(); // already in the main loop
  get_data(mouse_movement);
  hid_task(hid_sent_flag, 0); // make sure hid run the movement command
}


// void finger_loop(){
//   int imu_accel_[WINDOW_LENGTH*3];
//   int imu_gyro_[WINDOW_LENGTH*3];
//   int finger_imu_index_tail=WINDOW_LENGTH*3-1;
//   finger_update(imu_accel, imu_gyro,finger_indextail);
  
// }
// // imu_accel[WINDOW_LENGTH*3];
// // imu_gyro[WINDOW_LENGTH*3];
// void finger_update(int imu_accel*,int imu_gyro*,int &finger_indextail){
//   // static int imu_accel[3][WINDOW_LENGTH];
//   // static int imu_gyro[3][WINDOW_LENGTH];
//   static int window_index=0;
//   int16_t acceleration1[3], gyro1[3], temp1;
//   mpu6050_read_raw(0x68, acceleration1, gyro1, &temp1);
//   int filtered_accel1[3];
//   int filtered_gyro1[3];

//   float final_gyro1[3]; // output type
//   float final_accel1[3];

//   for(int i=0;i<3;i++){

//       // assign new value to window
//     imu1_accel[window_index+i]=(int)acceleration1[i];
//     imu1_gyro[window_index+i]=(int)gyro1[i];
//       // sum all (still in raw data(int16_t))
//     filtered_accel1[i]= sliding_window_filtering_pp(imu1_accel,WINDOW_LENGTH,);
//     filtered_gyro1[i]= sliding_window_filtering_pp(imu1_gyro,WINDOW_LENGTH);
//       // convert to final results
//     final_gyro1[i]=(float)filtered_gyro1[i]*250.0 / 32768.0;
//     final_accel1[i]=(float)filtered_accel1[i]*4.0 / 32768.0;
//   }    
//     window_index = (window_index+3)%WINDOW_LENGTH;
//     finger_indextail=window_index;
//     char str[200];
   
//     // sprintf(str, "%.3f,%.3f,%.3f,%.3f\n", 
//     //                                                       final_gyro1[1],final_gyro2[1],gyro1_[1],tmp3);
//     /*
//       final_accel1[0]   ---     final_accel2[0]
//       final_accel1[1]   ---     final_accel2[1]
//       final_gyro1[0]    ---     final_gyro2[0]
//       final_gyro1[1]    ---     final_gyro2[1]
//       final_gyro1[2]    ---     final_gyro2[2]

//     */
//     // sprintf(str, "%.3f,%.3f,%.3f,%.3f\n", final_accel1[1],final_accel2[0],final_accel2[1],final_accel2[2]);
// }


void screen_show(){
  // char str1[10], str2[10], str3[10];
  // sprintf(str1, "x: %.1f", current_position[0]);
  // sprintf(str2, "y: %.1f", current_position[1]);
  // sprintf(str3, "z: %.1f\n"), current_position[2];
  // ST7735_WriteString(1, 45, str1, Font_11x18, ST7735_BLACK, ST7735_GREEN);
  // ST7735_WriteString(1, 65, str2, Font_11x18, ST7735_BLACK, ST7735_GREEN);
  // ST7735_WriteString(1, 85, str3, Font_11x18, ST7735_BLACK, ST7735_GREEN);
  // uart_puts(uart0, str1);
  // uart_puts(uart0, str2);
  // uart_puts(uart0, str3);
}

void loop() {
  gpio_put(25, !gpio_get(25));
  int accelerometer_samples_read;
  int gyroscope_samples_read;

  ReadAccelerometerAndGyroscope(&accelerometer_samples_read, &gyroscope_samples_read);

  // Parse and process IMU data
  bool done_just_triggered = false;
  if (gyroscope_samples_read > 0) {
    //current_gyroscope_drift: angular change speed 
    EstimateGyroscopeDrift(current_gyroscope_drift);
    char str[100];
    sprintf(str, "\r\ndx: %.3f \tdy: %.3f\tdz: %.3f\t", current_gyroscope_drift[0],current_gyroscope_drift[1],current_gyroscope_drift[2]);
    // uart_puts(uart0, str);
    UpdateOrientation(gyroscope_samples_read, current_gravity, current_gyroscope_drift);
    // char str2[100];
    // UpdateOrientation(gyroscope_samples_read, current_gravity, current_gyroscope_drift,str2);  // test UpdateOrientation
    // uart_puts(uart0, str2);
    UpdateStroke(gyroscope_samples_read, &done_just_triggered);
#if 1
    if (linked) {
      if (first) {
        //        sleep_ms(5000);
        first = false;
      }
      if (send_index++ % 16 == 0) {
        uart_write_blocking(UART_ID, stroke_struct_buffer, 328);
      }
    }
    else {
      first      = true;
      send_index = 0;
    }
#endif
  }

  if (accelerometer_samples_read > 0) {
    EstimateGravityDirection(current_gravity);
    UpdateVelocity(accelerometer_samples_read, current_gravity);
  }
  // Wait for a gesture to be done
  if (done_just_triggered and !linked) {
    // Rasterize the gesture
    RasterizeStroke(stroke_points, *stroke_transmit_length, 0.6f, 0.6f, raster_width,
                    raster_height, raster_buffer);
    auto    *displayBuf = new uint8_t[96 * 96 * 2];
    uint16_t index      = 0;
    for (int y = 0; y < raster_height; ++y) {
      char line[raster_width + 1];
      for (int x = 0; x < raster_width; ++x) {
        const int8_t *pixel =
          &raster_buffer[(y * raster_width * raster_channels) + (x * raster_channels)];
        const int8_t red    = pixel[0];
        const int8_t green  = pixel[1];
        const int8_t blue   = pixel[2];
        char         output = '.';
        // default green
        uint16_t imageRGB = ST7735_COLOR565(0, 255, 0);
        if ((red > -128) || (green > -128) || (blue > -128)) {
          output = '#';
          // black
          imageRGB = ST7735_COLOR565(0, 0, 0);
        }
        line[x]             = output;
        displayBuf[index++] = (uint8_t)(imageRGB >> 8) & 0xFF;
        displayBuf[index++] = (uint8_t)(imageRGB)&0xFF;
      }
      line[raster_width] = 0;
      // char str[100] ;
      // sprintf(str,"%s\n",line);
      // uart_puts(uart0, str);
      //printf("%s\n", line);
    }

    // Pass to the model and run the interpreter
    TfLiteTensor *model_input = interpreter->input(0);
    for (int i = 0; i < raster_byte_count; ++i) {
      model_input->data.int8[i] = raster_buffer[i];
    }

    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
      return;
    }

    TfLiteTensor *output = interpreter->output(0);

    // Parse the model output
    int8_t max_score;
    int    max_index;
    for (int i = 0; i < label_count; ++i) {
      const int8_t score = output->data.int8[i];
      if ((i == 0) || (score > max_score)) {
        max_score = score;
        max_index = i;
      }
    }
    TF_LITE_REPORT_ERROR(error_reporter, "Found %s (%d%%)", labels[max_index],
                         ((max_score + 128) * 100) >> 8);
#if SCREEN

    // char str1[10], str2[10], str3[10];
    // //sprintf(str, "%s:%d%%", labels[max_index], ((max_score + 128) * 100) >> 8);
    // sprintf(str1, "x:");
    // sprintf(str2, "y:");
    // sprintf(str3, "z:");
    // // ST7735_FillRectangle(0, 90, ST7735_WIDTH, 160 - 90, ST7735_GREEN);
    // // ST7735_FillRectangle(23, 90, 34, 34, ST7735_BLACK);
    // // ST7735_DrawImage(24, 91, 32, 32, displayBuf);
    // ST7735_WriteString(15, 130, str1, Font_11x18, ST7735_BLACK, ST7735_GREEN);
#endif
    delete[] displayBuf;
  }
}

void click_detect(){
  float right_acc[3], right_gyro[3];
  float left_acc[3], left_gyro[3];
  float palm_acc[3], palm_gyro[3];
  mpu6050_read_data(0x69, right_acc, right_gyro);
  mpu6050_read_data(0x68, left_acc, left_gyro);
  ReadICM42622(palm_acc, palm_gyro);
  float v_right = VectorMagnitude(right_acc);
  float v_left = VectorMagnitude(left_acc);
  float v_palm = VectorMagnitude(palm_acc);
  bool temp_flag;
  if(v_right - v_palm > CLICK_THRES){
    hid_task(temp_flag, 2);
    uart_puts(uart0, "Right click");
  }
  if(v_left - v_palm > CLICK_THRES){
    hid_task(temp_flag, 1);
    uart_puts(uart0, "Left click");
  }
  // char str[40] ;
  // sprintf(str," %f  %f  %f\r\n", v_right, v_left, v_palm);
  // uart_puts(uart0, str);
}

void keyboard_loop(char & key) {
  gpio_put(25, !gpio_get(25));
  int accelerometer_samples_read;
  int gyroscope_samples_read;

  for(int i=0;i<3;++i){
    float acceleration1[3], gyro1[3];
    mpu6050_read_data(0x68, acceleration1, gyro1);
    ReadMPU6050(&accelerometer_samples_read, &gyroscope_samples_read, acceleration1, gyro1);
  }
  // Parse and process IMU data
  bool done_just_triggered = false;
  if (gyroscope_samples_read > 0) {
    //current_gyroscope_drift: angular change speed 
    EstimateGyroscopeDrift(current_gyroscope_drift);
    // char str[100];
    // sprintf(str, "\r\ndx: %.3f \tdy: %.3f\tdz: %.3f\t", current_gyroscope_drift[0],current_gyroscope_drift[1],current_gyroscope_drift[2]);
    // uart_puts(uart0, str);
    UpdateOrientation(gyroscope_samples_read, current_gravity, current_gyroscope_drift);
    UpdateStroke(gyroscope_samples_read, &done_just_triggered);
#if 1
    if (linked) {
      if (first) {
        //        sleep_ms(5000);
        first = false;
      }
      if (send_index++ % 16 == 0) {
        uart_write_blocking(UART_ID, stroke_struct_buffer, 328);
      }
    }
    else {
      first      = true;
      send_index = 0;
    }
#endif
  }

  if (accelerometer_samples_read > 0) {
    EstimateGravityDirection(current_gravity);
    UpdateVelocity(accelerometer_samples_read, current_gravity);
  }
  // Wait for a gesture to be done
  if (done_just_triggered and !linked) {
    // char print_1[100];
    // sprintf(print_1,"a gesture has done\n");
    // uart_puts(uart0,print_1);
    // Rasterize the gesture
    RasterizeStroke(stroke_points, *stroke_transmit_length, 0.6f, 0.6f, raster_width,
                    raster_height, raster_buffer);
    auto    *displayBuf = new uint8_t[96 * 96 * 2];
    uint16_t index      = 0;
    for (int y = 0; y < raster_height; ++y) {
      char line[raster_width + 1];
      for (int x = 0; x < raster_width; ++x) {
        const int8_t *pixel =
          &raster_buffer[(y * raster_width * raster_channels) + (x * raster_channels)];
        const int8_t red    = pixel[0];
        const int8_t green  = pixel[1];
        const int8_t blue   = pixel[2];
        char         output = '.';
        // default green
        uint16_t imageRGB = ST7735_COLOR565(0, 255, 0);
        if ((red > -128) || (green > -128) || (blue > -128)) {
          output = '#';
          // black
          imageRGB = ST7735_COLOR565(0, 0, 0);
        }
        line[x]             = output;
        displayBuf[index++] = (uint8_t)(imageRGB >> 8) & 0xFF;
        displayBuf[index++] = (uint8_t)(imageRGB)&0xFF;
      }
      line[raster_width] = 0;
      
      // printf("%s\n", line);
      // uart_puts(uart0, line);
            char str[100] ;
      sprintf(str,"%s\r\n",line);
      uart_puts(uart0, str);
    }

    // Pass to the model and run the interpreter
    TfLiteTensor *model_input = interpreter->input(0);
    for (int i = 0; i < raster_byte_count; ++i) {
      model_input->data.int8[i] = raster_buffer[i];
    }

    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
      return;
    }

    TfLiteTensor *output = interpreter->output(0);

    // Parse the model output
    int8_t max_score;
    int    max_index;
    for (int i = 0; i < label_count; ++i) {
      const int8_t score = output->data.int8[i];
      if ((i == 0) || (score > max_score)) {
        max_score = score;
        max_index = i;
      }
    }
    TF_LITE_REPORT_ERROR(error_reporter, "Found %s (%d%%)", labels[max_index],
                         ((max_score + 128) * 100) >> 8);
    key = labels[max_index][0];
    // labels[max_index]
#if SCREEN

    // char str1[10], str2[10], str3[10];
    // //sprintf(str, "%s:%d%%", labels[max_index], ((max_score + 128) * 100) >> 8);
    // sprintf(str1, "x:");
    // sprintf(str2, "y:");
    // sprintf(str3, "z:");
    // // ST7735_FillRectangle(0, 90, ST7735_WIDTH, 160 - 90, ST7735_GREEN);
    // // ST7735_FillRectangle(23, 90, 34, 34, ST7735_BLACK);
    // // ST7735_DrawImage(24, 91, 32, 32, displayBuf);
    // ST7735_WriteString(15, 130, str1, Font_11x18, ST7735_BLACK, ST7735_GREEN);
#endif
    delete[] displayBuf;
  }
}

// // created by PP
// void IMUupdate() {
//   // gpio_put(25, !gpio_get(25));
//   float accelerometer_samples_read[3]; // read signal, >0 only when acquire data  -pp
//   float gyroscope_samples_read[3];     // read signal, >0 only when acquire data  -pp
//   static float angular[2]={0};  // angular xz, angular yz -pp
//   ReadAccelerometerAndGyroscope(accelerometer_samples_read, gyroscope_samples_read,angular);
//   char str[100];
//   // sprintf(str, "\r\n\tx: %.3f\ty: %.3f\tz: %.3f", accelerometer_samples_read[0],accelerometer_samples_read[1],accelerometer_samples_read[2]);
//   sprintf(str, "\n%.3f\t%.3f\t%.3f", accelerometer_samples_read[0],accelerometer_samples_read[1],accelerometer_samples_read[2]);
//   // sprintf(str, "\r\n\tx: %.3f\ty: %.3f\tz: %.3f\t%.3f\t%.3f", accelerometer_samples_read[0],accelerometer_samples_read[1],accelerometer_samples_read[2],angular[0],angular[1]);
//   // sprintf(str, "\n%.3f\t%.3f", angular[0],angular[1]);
//   if(accelerometer_samples_read[0]!=0)
//   uart_puts(uart0, str);
// }
