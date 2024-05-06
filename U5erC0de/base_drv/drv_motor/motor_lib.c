#include "./base_drv/drv_motor/motor_lib.h"

/*
typedef struct
{
	uint16_t measure_min;		//	lower limit of encoder output value
	uint16_t measure_max;		//	upper limit of encoder output value
	uint8_t  reduction_ratio;	//	reduction ratio
	uint16_t tx_base_ID; 		//	identifier of the message received by the motor (sent by us)
	uint16_t rx_base_ID;	 	//	identifier of motor feedback (we receive) message
								//	rx_hander.StdId=base_ID+motor_ID,motor_ID starts from 1
} motor_type_t;
*/

// M2006 / C610
const _Motor DJI_motor_lib_M2006 = {
	.L = {0, 8191, 36, 0x200, 0x200},
	.H = {0, 8191, 36, 0x1ff, 0x200}};
/*说明:
	L,H							//	L represents C610 with ID 1~4, and H is ID 5~8
	reduction_ratio;			//	36:1
	tx_base_ID;					//	Data frame 8 * 8 bytes, for example:
								the first byte controls the high 8 bits currents of ID1 motor,
								and the seconed byte controls the low 8-bit part.
								The final values range from -10000 to 10000,
								corresponding to the actual currents of -+10A.
	rx_base_ID;					//	frequency is 1000 Hz, which can be modified by software
*/

// M3508 / C620
const _Motor DJI_motor_lib_M3508 = {
	.L = {0, 8191, 3591.f / 187.f, 0x200, 0x200},
	.H = {0, 8191, 3591.f / 187.f, 0x1ff, 0x200}};
/*说明
	L,H							//L represents C620 with ID 1~4, and H is ID 5~8
	reduction_ratio;			//in fact it is 3591.0/187.0 to 1, that is, 19.20320856...to 1
	tx_base_ID;					//	Data frame 8 * 8 bytes, for example:
								the first byte controls the high 8 bits current of ID1 motor,
								and the seconed byte controls the low 8-bit part.
								The final values range from -16384 to 16384,
								corresponding to the actual currents of -+20A.
	rx_base_ID;					//	frequency is 1000 Hz, which can be modified by software
*/

// GM6020
const _Motor DJI_motor_lib_GM6020 = {
	.L = {0, 8191, 1, 0x1ff, 0x204},
	.H = {0, 8191, 1, 0x2ff, 0x204}};
/*说明
	L,H						//	L represents the motor with ID 1~4d and H is ID 5~7 (yes, up to 7 motors)
	reduction_ratio			//	Default is 1
	tx_base_ID;				//	Data frame 8 * 8 bytes, for example:
								the first byte controls the high 8 bits voltage of ID1 motor,
								and the seconed byte controls the low 8-bit part.
								The final values range from -30000 to 30000,
								the corresponding actual voltage relationship is unknown...
	rx_base_ID;				//	The identifier of GM6020 of ID1 is 0x205, so here base_id is set to 0x204
*/
