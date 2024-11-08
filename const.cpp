
#define PI 3.14159265358979323846
#define micro_rollover_useconds 4294967295

// BLUE SIDE 1
// RED SIDE 2
int ideal_ball = 2;

// fixed values
float L = 0.472;                         // distance between two horizontal wheels
float B = 0.122;                        // distance between vertical wheel and horizontal wheel
float R = 0.029;                         // radius of free wheel
float N = 600;                          // pulse per revolution
float cm_per_tick = (2 * PI * R) / (N); // 2*pi*R/N where N is ppr 0.0523

// encoder variables
float enc_count[3] = {0,0,0};

// robot movement per loop
float odom_dx;
float odom_dy;
float odom_dw;

// coordinates wrt ref frame
float odom_x = 0.0;
float odom_y = 0.0;
float odom_wi = 0;
float odom_wf;

// encooder pins A and B
int ENCA[3] = {ENC1_A, ENC2_A, ENC3_A}; // white
int ENCB[3] = {ENC1_B, ENC2_B, ENC3_B}; // grsplit this files into multiple files meaningfullyeen

// //line sensors
// int linesensors[4] = {LS1_An, LS2_An, LS3_An, LS4_An}; //insert pin numbers here
// int linesensor_data[4] = {0,0,0,0};
// int line_val[4];
// int linesensor_active[4];
// int prev_linesensor_active[4];
int threshold = 0;

int prev_junc = 0;
int junc = -1;
int junc2 = 0;
int junc3 = 0;

// velocities
float odom_vx = 0;
float odom_vy = 0;
float odom_vw = 0;


float target_rpm[4];
int target_vx = 0;
int target_vy = 0;
int target_vw = 0;

float b = 1;
float matrix[4][3] = {{-1.41421, 1.41421, b},
                      {1.41421, 1.41421, b},
                      {-1.41421, 1.41421, -b},
                      {1.41421, 1.41421, -b}};

//pickNdrop 
// int drop = 0;
// int pick = 0;
// int silo = 0;
// int grip_state = 0;
// int grip_angle = 170;
// int release_angle = 134;
// int open_door = 90;
// int close_door = 0;
// int ball_found = 0;

// int grip_delay = 0;
// int drop_delay = 0;
// int picker_distance = 0;
// int picker_max_distance = 5500;

// int drop_ls_pin = Lim1;
// int pick_ls_pin = Lim2;
// int silo_ls_pin = Lim3;

int thres = 0;

// MD5 arm motor
// MD6 roller motor

//botstatus
int botStatus = 0;


//junction caliberation
int prev_j_counter = 0;
int j_counter = -1;

// Stores frequency read by the photodiodes
int redFrequency = 0;
int blueFrequency = 0;
int redFrequency2 = 0;
int blueFrequency2 = 0;
int diff = 0;
int diff2 = 0;
int redCounter = 0;
int blueCounter = 0;
int purpleCounter = 0;

// Threshold values for red and blue color detection
const int counter_max = 5;
const int redThreshold = 150;  // Adjust this value as needed
const int blueThreshold = 150; // Adjust this value as needed
int detected_ball_color = -1;
// blue 1
// red 2
// purple 3

const int bot_start_LED = 13;
int LED_cnt = 0;

// RETRY ZONE
int reset_button_pin = 10;
int retry_cnt = 0;
int reset_button = 0;
int prev_reset_button = 0;