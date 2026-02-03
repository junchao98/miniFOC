
#include "commands.h"
#include "commander.h"
#include "gd32f1x0.h"
#include "foc.h"
#include "pid.h"
#include "uart.h"
#include "motor.h"

#define NANOPRINTF_IMPLEMENTATION
#include "nanoprintf.h"

// callback function pointer definiton
typedef void (*CommandCallback)(char *); //!< command callback function pointer

CommandCallback call_list[20]; //!< array of command callback pointers - 20 is an arbitrary number
char call_ids[20];             //!< added callback commands
char *call_label[20];          //!< added callback labels
int call_count = 0;            //!< number callbacks that are subscribed

// Commander verbose display to the user type
enum VerboseMode {
    nothing = 0x00,         // display nothing - good for monitoring
    on_request = 0x01,      // display only on user request
    user_friendly = 0x02,   // display textual messages to the user
    machine_readable = 0x03 // display machine readable commands, matching commands to set each settings
};

// printing variables
uint8_t verbose = user_friendly; //!< flag signaling that the commands should output user understanable text
uint8_t decimal_places = 3;      //!< number of decimal places to be used when displaying numbers
char eol = '\n';                 //!< end of line sentinel character
bool echo = false;               //!< echo last typed character (for command line feedback)

#define F(string_literal) (string_literal)

void print_ch(char ch)
{
    uart_sendbyte(ch);
}
void print(char *message)
{
    npf_printf("%s", message);
}

void println(char *message)
{
    npf_printf("%s\n", message);
}
void printVerbose(char *message)
{
    if (verbose == user_friendly)
        print(message);
}

void printMachineReadable(char *msg)
{
    if (verbose == machine_readable)
        npf_printf("%s", msg);
}
void printlnMachineReadable(char *msg)
{
    if (verbose == machine_readable)
        npf_printf("%s\n", msg);
}
void printMachineReadable_ch(char msg)
{
    if (verbose == machine_readable)
        npf_printf("%c", msg);
}

void printError()
{
    println("err");
}

uint8_t isSentinel(char ch)
{
    if (ch == eol)
        return true;
    else if (ch == '\r') {
        printVerbose(F("Warn: \\r detected! \n"));
        return true; // lets still consider it to end the line...
    }
    return false;
}

bool isDigit(char c)
{
    return (c >= '0' && c <= '9');
}

double atof(const char *str) {
    double result = 0.0;
    double fraction = 0.0;
    int sign = 1;
    int divisor = 1;
    int has_fraction = 0;

    // 跳过前导空格
    while (*str == ' ') {
        str++;
    }

    // 处理可选的符号
    if (*str == '-' || *str == '+') {
        if (*str == '-') {
            sign = -1;
        }
        str++;
    }

    // 处理整数部分
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }

    // 处理小数点
    if (*str == '.') {
        str++;
        has_fraction = 1;
    }

    // 处理小数部分
    while (has_fraction && *str >= '0' && *str <= '9') {
        fraction = fraction * 10 + (*str - '0');
        divisor *= 10;
        str++;
    }

    result += fraction / divisor;
    return sign * result;
}

int atoi(const char *str) {
    int result = 0;
    int sign = 1;

    // 跳过前导空格
    while (*str == ' ') {
        str++;
    }

    // 处理可选的符号
    if (*str == '-' || *str == '+') {
        if (*str == '-') {
            sign = -1;
        }
        str++;
    }

    // 处理数字字符
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }

    return sign * result;
}

void pid(PID_Structure_t *pid, char *user_cmd)
{
    char cmd = user_cmd[0];
    bool GET = isSentinel(user_cmd[1]);
    float value = atof(&user_cmd[1]);

    switch (cmd) {
    case SCMD_PID_P: // P gain change
        printVerbose("P: ");
        if (!GET)
            pid->kp = value;
        npf_printf("%.2f\n", pid->kp);
        break;
    case SCMD_PID_I: // I gain change
        printVerbose("I: ");
        if (!GET)
            pid->ki = value;
        npf_printf("%.2f\n", pid->ki);
        break;
    case SCMD_PID_D: // D gain change
        printVerbose("D: ");
        if (!GET)
            pid->kd = value;
        npf_printf("%.2f\n", pid->kd);
        break;
    case SCMD_PID_RAMP: //  ramp change
        printError();
        break;
    case SCMD_PID_LIM: //  limit change
        printVerbose("limit: ");
        if (!GET) {
            pid->maximum = value;
            pid->minimum = -value;
        }
        npf_printf("%f\n", pid->maximum);
        break;
    default:
        printError();
        break;
    }
}

void target(char *user_cmd, char *separator)
{
    // if no values sent
    if (isSentinel(user_cmd[0])) {
        //TODO
        return;
    };

    float pos, vel, torque;
    char *next_value;
    float val = atof(user_cmd); //FIXME

    motor.target = val;
    switch (motor.FOC_Struct.control_mod) {
    case MCT_torque: // setting torque target
        torque = val;
        motor.FOC_Struct.user_expect = torque;
        break;
    case MCT_velocity: // setting velocity target + torque limit
        // set the target
        vel = val;
        motor.speed_pid_handler.expect = vel;
        break;
    case MCT_angle: // setting angle target + torque, velocity limit
        // setting the target position
        pos = val;
        motor.angle_pid_handler.expect = pos;
        break;
    case MCT_velocity_openloop: // setting velocity target + torque limit
        // set the target
        printError();
        break;
    case MCT_angle_openloop: // setting angle target + torque, velocity limit
        // set the target
        printError();
        break;
    }
    printVerbose(F("Target: "));
    npf_printf("%.2f\n", val);
}

void cmd_motion(char *user_cmd, char *separator)
{
    char cmd = user_cmd[0];
    char sub_cmd = user_cmd[1];
    bool GET = isSentinel(user_cmd[1]);
    float value = atof(&user_cmd[(sub_cmd >= 'A' && sub_cmd <= 'Z') ? 2 : 1]);

    switch (cmd) {
    case CMD_MOTION_TYPE:
        printVerbose(F("Motion:"));
        switch (sub_cmd) {
        case SCMD_DOWNSAMPLE:
            printVerbose(F(" downsample: "));
            if(!GET) motor.motion_downsample = value;
            npf_printf("%d\n",  motor.motion_downsample);
          break;

            break;
        default:
            // change control type
            if (!GET && value >= 0 && (int)value < 5) // if set command
                motor.FOC_Struct.control_mod = value;
            switch (motor.FOC_Struct.control_mod) {
            case MCT_torque:
                println(F("torque"));
                break;
            case MCT_velocity:
                println(F("vel"));
                break;
            case MCT_angle:
                println(F("angle"));
                break;
            case MCT_velocity_openloop:
                println(F("vel open"));
                break;
            case MCT_angle_openloop:
                println(F("angle open"));
                break;
            }
            break;
        }
        break;
    case CMD_TORQUE_TYPE:
        // change control type
        printVerbose(F("Torque: "));
        println(F("volt"));
        break;
    case CMD_STATUS:
        // enable/disable
        printVerbose(F("Status: "));
        if (!GET){
            motor.enabled = value;
            if(motor.enabled)
                motor_enable();
            else
                motor_disable();
        }
        npf_printf("%d\n", motor.enabled);
        break;
    default:
        target(user_cmd, separator);
        break;
    }
}

void cmd_motor( char *user_command)
{

    // if target setting
    if (isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+' || isSentinel(user_command[0])) {
        target(user_command, " ");
        return;
    }

    // parse command letter
    char cmd = user_command[0];
    char sub_cmd = user_command[1];
    // check if there is a subcommand or not
    int value_index = (sub_cmd >= 'A' && sub_cmd <= 'Z') || (sub_cmd == '#') ? 2 : 1;
    // check if get command
    bool GET = isSentinel(user_command[value_index]);
    // parse command values
    float value = atof(&user_command[value_index]);
    printMachineReadable(cmd);
    if (sub_cmd >= 'A' && sub_cmd <= 'Z') {
        printMachineReadable(sub_cmd);
    }

    // a bit of optimisation of variable memory for Arduino UNO (atmega328)
    switch (cmd) {
    case CMD_C_Q_PID: //
        printVerbose(F("PID curr q| "));
        if (sub_cmd == SCMD_LPF_TF)
            printError();
        else
            printError();
        break;
    case CMD_C_D_PID: //
        printVerbose(F("PID curr d| "));
        printError();
        break;
    case CMD_V_PID: //
        printVerbose(F("PID vel| "));
        if (sub_cmd == SCMD_LPF_TF)
            printError();
        else
            pid(pid_get_speed_pid(), &user_command[1]);
        break;
    case CMD_A_PID: //
        printVerbose(F("PID angle| "));
        if (sub_cmd == SCMD_LPF_TF)
            printError();
        else
            pid(pid_get_angle_pid(), &user_command[1]);
        break;
    case CMD_LIMITS: // TODO
        printVerbose(F("Limits| "));
        switch (sub_cmd) {
        case SCMD_LIM_VOLT: // voltage limit change
            printVerbose(F("volt: "));
            if (!GET) {
                ;
            }
            npf_printf("8.0\n");
            break;
        case SCMD_LIM_CURR: // current limit
            printVerbose(F("curr: "));
            if (!GET) {
                ;
            }
            npf_printf("1.0\n");
            break;
        case SCMD_LIM_VEL: // velocity limit
            printVerbose(F("vel: "));
            if (!GET) {
                ;
            }
            npf_printf("100.0\n");
            break;
        default:
            printError();
            break;
        }
        break;
    case CMD_MOTION_TYPE:
    case CMD_TORQUE_TYPE:
    case CMD_STATUS:
        cmd_motion(&user_command[0], " ");
        break;
    case CMD_PWMMOD:
        // PWM modulation change
        printVerbose(F("PWM Mod | "));
        switch (sub_cmd) {
        case SCMD_PWMMOD_TYPE: // zero offset
            printVerbose(F("type: "));
            if (!GET)
                motor.FOC_Struct.pwmmod = value;
            switch (motor.FOC_Struct.pwmmod) {
            case SinePWM:
                println(F("SinePWM"));
                break;
            case SpaceVectorPWM:
                println(F("SVPWM"));
                break;
            case Trapezoid_120:
                println(F("Trap 120"));
                break;
            case Trapezoid_150:
                println(F("Trap 150"));
                break;
            }
            break;
        case SCMD_PWMMOD_CENTER: // centered modulation
            printVerbose(F("center: "));
            npf_printf("%.2f\n", 5.0);
            break;
        default:
            printError();
            break;
        }
        break;
    case CMD_SENSOR:
      // Sensor zero offset
       printVerbose(F("Sensor | "));
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          printVerbose(F("offset: "));
          //if(!GET) motor->sensor_offset = value;
          npf_printf("%.2f\n", 0.00);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          printVerbose(F("el. offset: "));
          //if(!GET) motor->zero_electric_angle = value;
          npf_printf("%.2f\n", 0.00);
          break;
        default:
          printError();
          break;
       }
      break;
    case CMD_MONITOR: // get current values of the state variables
        printVerbose(F("Monitor | "));
        switch (sub_cmd) {
        case SCMD_GET: // get command
            switch ((uint8_t)value) {
            case 0: // get target
                printVerbose(F("target: "));
                npf_printf("%.3f\n", motor.target);
                break;
            case 1: // get voltage q
              printVerbose(F("Vq: "));
              println("0.000");
              break;
            case 2: // get voltage d
              printVerbose(F("Vd: "));
              println("0.000");
              break;
            case 3: // get current q
              printVerbose(F("Cq: "));
              println("0.000");
              break;
            case 4: // get current d
              printVerbose(F("Cd: "));
              println("0.000");
              break;
            case 5: // get velocity
                printVerbose(F("vel: "));
                npf_printf("%.3f\n", motor.FOC_Struct.rotate_speed);
                break;
            case 6: // get angle
                printVerbose(F("angle: "));
                npf_printf("%.3f\n", motor.FOC_Struct.mechanical_angle);
                break;
            case 7: // get all states
                printVerbose(F("all: "));
                npf_printf("%.3f", motor.FOC_Struct.user_expect);
                print(";");
                print("0;0;0;0;");
                npf_printf("%.3f", motor.FOC_Struct.rotate_speed);
                print(";");
                npf_printf("%.3f\n", motor.FOC_Struct.mechanical_angle);
                break;
            default:
                //printError();
                break;
            }
            break;
        case SCMD_DOWNSAMPLE:
          printVerbose(F("downsample: "));
          if(!GET) motor.monitor_downsample = value;
          npf_printf("%d\n", motor.monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor.monitor_variables = (uint8_t) 0;
          println(F("clear"));
          break;
        case CMD_DECIMAL:
          printVerbose(F("decimal: "));
          //motor->monitor_decimals = value;
          npf_printf("%d\n", (int)value);
          break;
        case SCMD_SET:
          if(!GET){
            // set the variables
            motor.monitor_variables = (uint8_t) 0;
            for(int i = 0; i < 7; i++){
              if(isSentinel(user_command[value_index+i])) break;
              motor.monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
            }
          }
          // print the variables
          for(int i = 0; i < 7; i++){
            npf_printf("%d", (motor.monitor_variables & (1 << (6-i))) >> (6-i));
          }
          println("");
          break;

        default:
            printError();
            break;
        }
        break;
    default: // unknown cmd
        printVerbose(F("unknown cmd "));
        printError();
    }
}

void command_handle(char *user_input)
{
    char tx_buf[32] = {0};
    char id = user_input[0];
    switch (id) {
    case CMD_SCAN:
        for (int i = 0; i < call_count; i++) {
            printMachineReadable_ch(CMD_SCAN);

            print_ch(call_ids[i]);
            print(":");
            if (call_label[i])
                println(call_label[i]);
            else
                println("");
        }
        break;
    case CMD_VERBOSE:
        if (!isSentinel(user_input[1]))
            verbose = atoi(&user_input[1]);
        printVerbose(F("Verb:"));
        printMachineReadable_ch(CMD_VERBOSE);
        switch (verbose) {
        case nothing:
            println(F("off!"));
            break;
        case on_request:
        case user_friendly:
            println(F("on!"));
            break;
        case machine_readable:
            printlnMachineReadable(F("machine"));
            break;
        }
        break;
    case CMD_DECIMAL:
        if (!isSentinel(user_input[1]))
            decimal_places = atoi(&user_input[1]);
        printVerbose(F("Decimal:"));
        printMachineReadable_ch(CMD_DECIMAL);
        npf_printf("%c\n", decimal_places);
        break;
    default:
        cmd_motor(&user_input[1]);
        break;
        for (int i = 0; i < call_count; i++) {
            if (id == call_ids[i]) {
                printMachineReadable_ch(user_input[0]);
                call_list[i](&user_input[1]);
                break;
            }
        }
        break;
    }
}
