%% SIMULATION PARAMETERS
TIME_STEP = 32;             %[ms]

%% ROBOT PARAMETERS
ROBOT_NAME = 'TableUV';
ROBOT_BASE_RADIUS = 0.03;   %[m]
ROBOT_BASE_HEIGHT = 0.035;  %[m]
WHEEL_RADIUS = 0.017;       %[m]
WHEEL_FROM_CENTER = 0.04;   %[m]
WHEEL_CIRCUM = WHEEL_RADIUS * 2 * pi;

TABLE_WIDTH = 1.22;         %[m]
TABLE_HEIGHT = 0.76;        %[m]

MAX_SPEED = 3;              %[m/s]

%% SENSOR PARAMETERS
% ENCODER 
ENC_RESOLUTION = 0.1;      %[rad]
ENC_NOISE = 0.05;           %[rad]
ENC_UNIT = WHEEL_CIRCUM / 6.28;

% IMU_GYRO
IMU_GYRO_RESOLUTION = -1;
IMU_GYRO_NOISE = 0;

% IMU_ACC
IMU_ACC_RESOLUTION = -1;

% TOF
TOF_FOV = 0.471;            %[rad]
TOF_WIDTH = 64;             %[px]
TOF_HEIGHT = 64;            %[px]
TOF_IS_SPHERICAL = false;
TOF_NEAR = 0.01;            %[m]
TOF_MIN_RANGE = 0.01;       %[m]
TOF_MAX_RANGE = 4;          %[m]
TOF_MOTION_BLUR = 0;
TOF_NOISE = 0;
TOF_RESOLUTION = -1;







