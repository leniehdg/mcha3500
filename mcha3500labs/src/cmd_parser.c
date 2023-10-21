#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <inttypes.h> // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "cmd_line_buffer.h"
#include "cmd_parser.h"
#include "IMU.h"
#include "encoder.h"
#include "observer.h"
#include "controller.h"


// Type for each command table entry
typedef struct
{
    void (*func)(int argc, char *argv[]);   // Command function pointer
    const char * cmd;                       // Command name
    const char * args;                      // Command arguments syntax
    const char * help;                      // Command description
} CMD_T;

// Forward declaration for built-in commands
static void _help(int, char *[]);
static void _reset(int, char *[]);
static void dataLogging(int, char *[]);
static void IMU_Logging(int, char *[]);
static void encoder_logging(int, char *[]);
void obs_update(int argc, char *argv[]);
static void control_update(int, char *[]);


// Modules that provide commands
#include "heartbeat_cmd.h"

// Command table
static CMD_T cmd_table[] =
{
    {_help                       , "help"        , ""                   , "Displays this help message"                           },
    {_reset                      , "reset"       , ""                   , "Restarts the system."                                 },
    {heartbeat_cmd               , "heartbeat"   , "[start|stop]"       , "Get status or start/stop heartbeat task"              },
    {dataLogging                 , "logData"     , ""                   , "Displays the potentiometer voltage level."            },
    {IMU_Logging                 , "logIMU"      , ""                   , "Displays the IMU values level.           "            },
    {encoder_logging             , "logEncoder"  , ""                   , "Displays the Encoder value.              "            },
    {obs_update                  , "updateOb"    , "yi"                 , "Displays the angle and angular velocity of observer"  },
    {control_update              , "updateCtrl"  , ""                   , "Gets control values"                                  }, 

};
enum {CMD_TABLE_SIZE = sizeof(cmd_table)/sizeof(CMD_T)};
enum {CMD_MAX_TOKENS = 5};      // Maximum number of tokens to process (command + arguments)

// Command function definitions

static void control_update (int argc, char *argv[])
{
    // Check for correct input arguments
    if (argc != 3)
    {
        printf("Incorrect arguments\n");
    }
    else
    {
        // Convert parsed strings into floats
        float x1 = atof(argv[1]);
        float x2 = atof(argv[2]);
        
        /* Update states using ctrl_set_xi functions */
        ctrl_set_x1_int(x1);
        ctrl_set_x2_int(x2);

        /* Update controller value */
        ctrl_update();

        /* Print control action */
        // printf("%f\n", getControl());
    }
}


void obs_update (int argc, char *argv[])
{
    if (argc != 3)
	{
		printf("Incorrect number of arguments\n");
	}

	float yi = atof(argv[1]);
	float yi2 = atof(argv[2]);
	
	// observer_update(yi,yi2);
	
    // return the two states stored in xhm (predicted states after time step)
    float x1h = observer_get_dtheta();
	float x2h = observer_get_theta();

    // printf("dtheta: %f, theta: %f\n",x1h,x2h);

}


void IMU_Logging(int argc, char *argv[])
{
    UNUSED(argv);

    // logging_start();

}

void encoder_logging(int argc, char *argv[])
{
    UNUSED(argv);

    // logging_start();

}

void dataLogging(int argc, char *argv[])
{
    UNUSED(argv);

    // logging_start();

}


static void _print_chip_pinout(void);

void _help(int argc, char *argv[])
{
    UNUSED(argv);
    printf(
        "\n"
        "\n"
    );

    _print_chip_pinout();

    printf("\n");

    // Describe argument syntax using POSIX.1-2008 convention
    // see http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/V1_chap12.html
    switch (argc)
    {
    case 1:
        printf(
            "   Command Arguments            Description\n"
            "-------------------------------------------\n"
        );
        for (int i = 0; i < CMD_TABLE_SIZE; i++)
        {
            printf("%10s %-20s %s\n", cmd_table[i].cmd, cmd_table[i].args, cmd_table[i].help);
        }
        // printf("\nFor more information, enter help followed by the command name\n\n");
        break;
    case 2:
        printf("Not yet implemented.\n\n");
        // TODO: Scan command table, and lookup extended help string.
        break;
    default:
        printf("help is expecting zero or one argument.\n\n");
    }
}

void _reset(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
    // Reset the system
    HAL_NVIC_SystemReset();
}

void _print_chip_pinout(void)
{
    printf(
        "Pin configuration:\n"
        "\n"
        "       .---------------------------------------.\n"
        " PC10--|  1  2 --PC11              PC9--  1  2 |--PC8\n"
        " PC12--|  3  4 --PD2               PB8--  3  4 |--PC6\n"
        "  VDD--|  5  6 --E5V               PB9--  5  6 |--PC5\n"
        "BOOT0--|  7  8 --GND              AVDD--  7  8 |--U5V\n"
        "   NC--|  9 10 --NC                GND--  9 10 |--NC\n"
        "   NC--| 11 12 --IOREF             PA5-- 11 12 |--PA12\n"
        " PA13--| 13 14 --RESET             PA6-- 13 14 |--PA11\n"
        " PA14--| 15 16 --+3v3              PA7-- 15 16 |--PB12\n"
        " PA15--| 17 18 --+5v               PB6-- 17 18 |--NC\n"
        "  GND--| 19 20 --GND               PC7-- 19 20 |--GND\n"
        "  PB7--| 21 22 --GND               PA9-- 21 22 |--PB2\n"
        " PC13--| 23 24 --VIN               PA8-- 23 24 |--PB1\n"
        " PC14--| 25 26 --NC               PB10-- 25 26 |--PB15\n"
        " PC15--| 27 28 --PA0               PB4-- 27 28 |--PB14\n"
        "  PH0--| 29 30 --PA1               PB5-- 29 30 |--PB13\n"
        "  PH1--| 31 32 --PA4               PB3-- 31 32 |--AGND\n"
        " VBAT--| 33 34 --PB0              PA10-- 33 34 |--PC4\n"
        "  PC2--| 35 36 --PC1               PA2-- 35 36 |--NC\n"
        "  PC3--| 37 38 --PC0               PA3-- 37 38 |--NC\n"
        "       |________                   ____________|\n"
        "                \\_________________/\n"
    );
}

// Command parser and dispatcher

static int _makeargv(char *s, char *argv[], int argvsize);

#ifdef NO_LD_WRAP
void cmd_parse(char *) __asm__("___real_cmd_parse");
#endif

void cmd_parse(char * cmd)
{
    if (cmd == NULL)
    {
        printf("ERROR: Tried to parse NULL command pointer\n");
        return;
    }
    else if (*cmd == '\0') // Empty command string
    {
        return;
    }

    // Tokenise command string
    char *argv[CMD_MAX_TOKENS];
    int argc = _makeargv(cmd, argv, CMD_MAX_TOKENS);

    // Execute corresponding command function
    for (int i = 0; i < CMD_TABLE_SIZE; i++)
    {
        if (strcmp(argv[0], cmd_table[i].cmd) == 0)
        {
            cmd_table[i].func(argc, argv);
            return;
        }
    }

    // Command not found
    printf("Unknown command: \"%s\"\n", argv[0]);
}

// Command tokeniser

int _makeargv(char *s, char *argv[], int argvsize)
{
    char *p = s;
    int argc = 0;

    for(int i = 0; i < argvsize; ++i)
    {
        // skip leading whitespace
        while (isspace(*p))
            p++;

        if(*p != '\0')
            argv[argc++] = p;
        else
        {
            argv[argc] = NULL;
            break;
        }

        // scan over arg
        while(*p != '\0' && !isspace(*p))
            p++;

        // terminate arg
        if(*p != '\0' && i < argvsize - 1)
            *p++ = '\0';
    }

    return argc;
}

