// PINMAP
#define PPM_INT     40

#define MTR_PUSH     2  // PUSHER   // TODO : matching check
#define MTR_AILERON  4  // ROLL     // TODO : matching check
#define MTR_ELEVATOR 27 // PITCH    // TODO : matching check
#define MTR_RUDDER   26 // YAW      // TODO : matching check

#define LED_1   32
#define LED_2   33
#define LED_3   25
#define BUZ     15
#define MNT     36

// ETC
#define PPM_CH      8
#define PPM_IDX_A   0
#define PPM_IDX_E   1
#define PPM_IDX_T   2
#define PPM_IDX_R   3
#define PPM_IDX_ARM 6

#define ADDR_MPU9250 0x68

// Dynamic Parameter
#define GAIN_R_Kp 1
#define GAIN_R_Ki 0
#define GAIN_R_Kd 0
#define ITG_LIMIT_R 0

#define GAIN_P_Kp 1
#define GAIN_P_Ki 0
#define GAIN_P_Kd 0
#define ITG_LIMIT_P 0

#define GAIN_Y_Kp 0
#define GAIN_Y_Ki 0
#define GAIN_Y_Kd 0
#define ITG_LIMIT_Y 0

#define CTRL_MAX_R 15
#define CTRL_MAX_P 15
#define CTRL_MAX_Y 15
