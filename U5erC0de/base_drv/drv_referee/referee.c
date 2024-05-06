#include "./base_drv/drv_referee/referee.h"
#include "./algorithm/crc.h"
#include "./base_drv/drv_uart.h"

// #include "./robot_core/interface/interface_BTB.h"
#include <string.h>

// we only use the length of REFEREE_RX_BUFFER_LEN as a circular buffer,
// the remaining REFEREE_RX_BUFFER_LEN bytes is to solve the fragments in the
// circular buffer
/**
 * 	DMA设置成循环模式下可能会出现回绕使数据变得不连续
 *	此时就需要将前一段数据移到后一段的后面，因此预先分配两倍DMA缓冲区长度
 **/
extern referee_info_t referee_info;
uint8_t referee_rx_buffer[REFEREE_RX_BUFFER_LEN * 2];
uint16_t referee_rx_lost = REFEREE_SYS_MAX_LOST;

uint8_t referee_tx_buffer[REFEREE_TX_BUFFER_LEN];

ext_game_robot_status_t game_robot_status; // 0x0201
ext_game_status_t game_status;             // 0x0001
ext_game_robot_HP_t game_robot_HP;         // 0x0003
ext_robot_hurt_t robot_hurt;               // 0x0206
ext_shoot_data_t shoot_data;               // 0x0207
ext_power_heat_data_t power_heat_data;     // 0x0202
ext_rfid_status_t rfid_status;             // 0x0209
ext_game_robot_pos_t game_robot_pos;       // 0x0203

#define CMDID_TABLE_COL 11U
#define CMDID_TABLE_ROW 4U

/**
 * @var cmdid_buffer_table
 * @brief table(refer to pdf) find by cmdid
 * @note NULL = unused data, 0 = non-exist data
 */
/*裁判系统帧缓冲区指针表，从左到右依次为0-10，对应CMDID的低位
每列1-3对应CMDID高位，其中0为不存在的数据，而NULL则为存在但未使用的数据
在新增数据是需要定义变量并修改该表即可
*/
void *cmdid_buffer_table[CMDID_TABLE_ROW][CMDID_TABLE_COL] = {
    {0, &game_status, &game_robot_HP, NULL, NULL, NULL, 0, 0, 0, 0, 0},
    {0, NULL, NULL, NULL, NULL, 0, 0, 0, 0, 0, 0},
    {0, &game_robot_status, &power_heat_data, &game_robot_pos, NULL, NULL,
     &robot_hurt, &shoot_data, NULL, &rfid_status, NULL},
};

uint8_t cmdid_len_table[4][11] = {
    {0, sizeof(ext_game_status_t), sizeof(ext_game_result_t),
     sizeof(ext_game_robot_HP_t), 0, sizeof(ext_ICRA_buff_debuff_zone_status_t),
     0, 0, 0, 0, 0},
    {0, sizeof(ext_event_data_t), sizeof(ext_supply_projectile_action_t), 0,
     sizeof(ext_referee_warning_t), sizeof(ext_dart_remaining_time_t), 0, 0, 0,
     0, 0},
    {0, sizeof(ext_game_robot_status_t), sizeof(ext_power_heat_data_t),
     sizeof(ext_game_robot_pos_t), sizeof(ext_buff_t),
     sizeof(aerial_robot_energy_t), sizeof(ext_robot_hurt_t),
     sizeof(ext_shoot_data_t), sizeof(ext_bullet_remaining_t),
     sizeof(ext_rfid_status_t), sizeof(ext_dart_client_cmd_t)},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
#if 0
//normal版本,测试用
//用前把referee_rx_buffer长度调回REFEREE_RX_BUFFER_LEN,省点内存
uint32_t read_referee_data(uint8_t * buf,uint32_t start_offset, uint32_t len);

extern UART_HandleTypeDef REFEREE_UART_HANDLE;
uint16_t rx_len=0;
HAL_StatusTypeDef referee_recv_dma_init(void)
{
	HAL_StatusTypeDef result = uart_recv_dma_init(&(REFEREE_UART_HANDLE),referee_rx_buffer,REFEREE_RX_BUFFER_LEN);
	__HAL_UART_ENABLE_IT(&REFEREE_UART_HANDLE,UART_IT_IDLE);
	return HAL_UART_Receive_DMA(&REFEREE_UART_HANDLE,referee_rx_buffer,REFEREE_RX_BUFFER_LEN);
}

void referee_uart_idle_handle(UART_HandleTypeDef *huart)
{
	if(	huart==&REFEREE_UART_HANDLE
	&&	__HAL_UART_GET_FLAG(&REFEREE_UART_HANDLE,UART_FLAG_IDLE)!=RESET )
	{
		__HAL_UART_CLEAR_IDLEFLAG(&REFEREE_UART_HANDLE);									
		HAL_UART_DMAStop(&REFEREE_UART_HANDLE);
		
		extern DMA_HandleTypeDef hdma_usart6_rx;
		rx_len=REFEREE_RX_BUFFER_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
		read_referee_data(referee_rx_buffer,0,rx_len);
		
		HAL_UART_Receive_DMA(&REFEREE_UART_HANDLE,referee_rx_buffer,REFEREE_RX_BUFFER_LEN);
	}
}
frame_header_t fh={0};
uint16_t cur_frame_len=0;
uint8_t cmdid_h=0;
uint8_t cmdid_l=0;
uint32_t read_referee_data(uint8_t* buf,uint32_t start_offset, uint32_t len)
{
	//fh = *((frame_header_t*)buf);								//寄的
	//memcpy(&fh,&buf[0],sizeof(frame_header_t));	//寄的

	fh.sof=buf[0];
	fh.data_len=(uint16_t)(((uint16_t)buf[2]<<8)|buf[1]);
	fh.seq=buf[3];
	fh.crc8=buf[4];

	//查找帧头字节并校验帧头
	if(	fh.sof == SOF_BYTE
	&&	Verify_CRC8_Check_Sum(buf,LEN_HEADER))
	{
		cur_frame_len = LEN_HEADER + LEN_CMDID + fh.data_len + LEN_TAIL;
		//提取CMDID高位和低位
//		cmdid_l = buf[CMDID_OFFSET]-1;
//		cmdid_h = buf[CMDID_OFFSET+1];
			cmdid_l = buf[CMDID_OFFSET];								//低位信息发送为从0x01开始,转换到数组上就得减一
			cmdid_h = buf[CMDID_OFFSET + 1];
		//查指针表和数据长度表并校验
		if(	cmdid_h < CMDID_TABLE_ROW 
		&& 	cmdid_l < CMDID_TABLE_COL
		&& 	cmdid_buffer_table[cmdid_h][cmdid_l] != NULL 
		&& 	fh.data_len == cmdid_len_table[cmdid_h][cmdid_l])
		{						//校验数据部分CRC
			if(Verify_CRC16_Check_Sum(buf,cur_frame_len))
			{
				memcpy(cmdid_buffer_table[cmdid_h][cmdid_l]
					, &buf[LEN_HEADER + LEN_CMDID]
					,cmdid_len_table[cmdid_h][cmdid_l]);
			}
		}
	}
	return 0;
}
#else
// circular版本
// 数据接收标识符,第n列数组的第m位bit置1,则代表收到ID为0x0n0m的数据
uint16_t cmdid_rx_table[CMDID_TABLE_ROW];
//	uint8_t acc[10];
extern UART_HandleTypeDef REFEREE_UART_HANDLE;
bool aaa[6] = {0};
/**
 * @brief      enable referee receive uart it and initialize DMA
 * @return     set HAL_OK or HAL_BUSY
 */
HAL_StatusTypeDef referee_recv_dma_init(void) {
  memset(cmdid_rx_table, 0, CMDID_TABLE_ROW * sizeof(uint16_t));

  HAL_StatusTypeDef result = uart_recv_dma_init(
      &(REFEREE_UART_HANDLE), referee_rx_buffer, REFEREE_RX_BUFFER_LEN);
  __HAL_UART_ENABLE_IT(&(REFEREE_UART_HANDLE), UART_IT_IDLE);
  return result;
}

// static volatile uint32_t shoot_delay_cnt = 0; //delay in us

/**
  * @brief  read referee system data to structs
  * @param[in]  buf            the circular dma buffer
  * @param[in]  start_offset   position start to read
  * @param[in]  len            maximum bytes to read
  * @return the end offset of successfully processed data, start to read buffer
  from this next time
  * @attention  circular buffer is not supported,
                                make sure that start_offset + len <
  REFEREE_RX_BUFFER_LEN*2
  */
frame_header_t fh = {0};
uint8_t cmdid_h;
uint8_t cmdid_l;
uint8_t flag = 0;
uint32_t next_start, cur_start, cur_frame_len;
uint32_t read_referee_data(uint8_t *buf, uint32_t start_offset, uint32_t len) {

  uint32_t max_offset = len + start_offset;
  for (cur_start = start_offset; cur_start < max_offset;
       cur_start = next_start) {
    // verify fragment before read
    // 是否读取到缓冲区末尾
    if (LEN_HEADER + cur_start > max_offset)
      return cur_start;
    // fh = (frame_header_t*)&buf[cur_start];
    fh.sof = buf[cur_start];
    fh.data_len =
        (uint16_t)(((uint16_t)buf[cur_start + 2] << 8) | buf[cur_start + 1]);
    fh.seq = buf[cur_start + 3];
    fh.crc8 = buf[cur_start + 4];

    // 查找帧头字节并校验帧头
    if (fh.sof == SOF_BYTE &&
        Verify_CRC8_Check_Sum(&buf[cur_start], LEN_HEADER)) {
      cur_frame_len = LEN_HEADER + LEN_CMDID + fh.data_len + LEN_TAIL;
      // verify fragment before read
      // 是否读取到缓冲区末尾
      if (cur_frame_len + cur_start > max_offset) {
        return cur_start;
      }
      // 提取CMDID高位和低位
      cmdid_l =
          buf[cur_start +
              CMDID_OFFSET]; // 低位信息发送为从0x01开始,转换到数组上就得减一
      cmdid_h = buf[cur_start + CMDID_OFFSET + 1];
      if (cmdid_h == 0x02 && cmdid_l == 0x07)
        flag = 1;
      // 查指针表和数据长度表并校验
      aaa[0] = cmdid_h < CMDID_TABLE_ROW;
      aaa[1] = cmdid_l < CMDID_TABLE_COL;
      aaa[2] = cmdid_buffer_table[cmdid_h][cmdid_l] != NULL;
      aaa[3] = fh.data_len == cmdid_len_table[cmdid_h][cmdid_l];

      if (aaa[0] & aaa[1] & aaa[2] & aaa[3]) {
        // 校验数据部分CRC
        if (Verify_CRC16_Check_Sum(&buf[cur_start], cur_frame_len)) {
          memcpy(cmdid_buffer_table[cmdid_h][cmdid_l],
                 &buf[cur_start + LEN_HEADER + LEN_CMDID],
                 cmdid_len_table[cmdid_h][cmdid_l]);
          cmdid_rx_table[cmdid_h] |= ((uint16_t)1 << cmdid_l);
          // /*
          // if(cmdid_h==0x02&&cmdid_l==0x07)
          // {
          // 	union
          // 	{
          // 		uint8_t _u8[4];
          // 		float _flt;
          // 	}a;
          // 	a._u8[0]=buf[cur_start + LEN_HEADER + LEN_CMDID+3];
          // 	a._u8[1]=buf[cur_start + LEN_HEADER + LEN_CMDID+4];
          // 	a._u8[2]=buf[cur_start + LEN_HEADER + LEN_CMDID+5];
          // 	a._u8[3]=buf[cur_start + LEN_HEADER + LEN_CMDID+6];

          // 	shoot_data.bullet_speed=a._flt;
          // }
          // */
        }
      }
      // going into next frame
      // 将偏移移到该帧末尾，继续循环
      next_start = cur_frame_len + cur_start;
    } else {
      // going into next byte to find SOF
      // 每次移一个字节查找帧头字节
      next_start = cur_start + 1;
    }
  }
  return cur_start;
}

/**
 * @brief idle interrupt of referee uart
 * @param  UART_HandleTypeDef *huart struct of UART
 * @return void
 */
uint32_t previous_offset = 0;
uint32_t current_offset, len;
void referee_uart_idle_handle(UART_HandleTypeDef *huart) {
  if (huart->Instance == REFEREE_UART &&
      __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    current_offset = REFEREE_RX_BUFFER_LEN - huart->hdmarx->Instance->NDTR;
    if (current_offset > previous_offset) {
      // 缓冲区数据连续，直接给出长度
      len = current_offset - previous_offset;
      // previous_offset =
      // read_referee_data(referee_rx_buffer,previous_offset,len);
    } else {
      // if data circle back from end to start, copy the fragment into
      // continuous buffer
      // 缓冲区数据不连续，出现回绕，计算出正确长度后将前半段数据复制到后半段的正后方
      len = REFEREE_RX_BUFFER_LEN - previous_offset + current_offset;
      memcpy(&referee_rx_buffer[REFEREE_RX_BUFFER_LEN], referee_rx_buffer,
             current_offset);
    }
    previous_offset =
        (read_referee_data(referee_rx_buffer, previous_offset, len)) %
        REFEREE_RX_BUFFER_LEN;
    // previous_offset = current_offset;
  }
}

/**
 * @brief      send a single frame to referee system(ui, custom data etc.)
 * @param[in]  huart   	pointer to the uart handle
 * @param[in]  cmdid       cmdid of data to send
 * @param[in]  p_struct   	pointer to the struct to send
 * @param[in]  len   		data length
 * @return     size of the total data len, zero for buffer over flow
 */
/*裁判系统发送数据*/
uint16_t send_referee_single(UART_HandleTypeDef *huart, uint16_t cmd_id,
                             uint8_t *p_struct, uint16_t len) {
  uint16_t total_size;
  frame_header_t tx_header;

  if (len + sizeof(frame_header_t) + 4 > REFEREE_TX_BUFFER_LEN)
    return 0;
  memset(referee_tx_buffer, 0, REFEREE_TX_BUFFER_LEN);

  tx_header.sof = SOF_BYTE;
  tx_header.data_len = len;
  tx_header.seq = 0;
  tx_header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&tx_header,
                                      sizeof(frame_header_t) - 1, 0xff);
  memcpy(&referee_tx_buffer, &tx_header, sizeof(frame_header_t));

  *(uint16_t *)&referee_tx_buffer[sizeof(frame_header_t)] = cmd_id;
  memcpy(&referee_tx_buffer[sizeof(frame_header_t) + sizeof(uint16_t)],
         p_struct, len);

  total_size =
      sizeof(frame_header_t) + sizeof(uint16_t) + len + sizeof(uint16_t);
  Append_CRC16_Check_Sum(referee_tx_buffer, total_size);

#if REFEREE_ASYNC_SEND == 0
  uart_send_async(huart, referee_tx_buffer, total_size);
#else
  uart_send(huart, referee_tx_buffer, total_size);
#endif

  return total_size;
}
/**
 * @brief  get rx flags of specific cmdid
 * @param[in]  cmdid       CMDID of data
 * @return non-zero -> already receive, zero -> not receive
 */
uint16_t get_referee_rx_flags(uint16_t cmdid) {
  uint8_t cmd_id_l = cmdid & 0xFF;
  uint8_t cmd_id_h = cmdid >> 8;
  if (cmd_id_h >= CMDID_TABLE_ROW || cmd_id_l >= CMDID_TABLE_COL) {
    return 0;
  } else {
    return (cmdid_rx_table[cmd_id_h] & ((uint16_t)1 << cmd_id_l));
  }
}

/**
 * @brief  clear rx flags of specific cmdid
 * @param[in]  cmdid       CMDID of data
 * @return none
 */
void clear_referee_rx_flags(uint16_t cmdid) {
  uint8_t cmd_id_l = cmdid & 0xFF;
  uint8_t cmd_id_h = cmdid >> 8;
  if (cmd_id_h < CMDID_TABLE_ROW && cmd_id_l < CMDID_TABLE_COL) {
    cmdid_rx_table[cmd_id_h] &= ~((uint16_t)1 << cmd_id_l);
  }
}

/**
 * @brief  increment of referee_rx_lost counter, should be call after process of
 * control
 * @return none
 */
void inc_referee_rx_lost(void) {
  if (get_referee_rx_flags(REFEREE_TEST_CMDID)) {
    clear_referee_rx_flags(REFEREE_TEST_CMDID);
    referee_rx_lost = 0;
  } else {
    if (referee_rx_lost < REFEREE_SYS_MAX_LOST)
      referee_rx_lost++;
  }
}

/**
 * @brief  check if referee system is offline
 * @return 0 for referee system online, others for offline
 */
uint32_t is_referee_sys_offline(void) {
  return (referee_rx_lost >= REFEREE_SYS_MAX_LOST);
}

/**
 * @brief clear referee_rx_buffer and all data type buffer
 * @return none
 */
void referee_buffer_clr(void) {
  int i, j;
  memset(referee_rx_buffer, 0, REFEREE_RX_BUFFER_LEN * 2);
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 10; j++) {
      if (cmdid_buffer_table[i][j] != NULL) {
        memset(cmdid_buffer_table[i][j], 0, cmdid_len_table[i][j]);
      }
    }
  }
}

#endif

void update_referee_data_to_BTB(void) {
  memcpy(&referee_info.robot_id, &game_robot_status.robot_id, 5);

  referee_info.bullet_freq = shoot_data.bullet_freq;
  referee_info.bullet_speed = shoot_data.bullet_speed;
  referee_info.shooter_barrel_heat_limit =
      game_robot_status.shooter_barrel_heat_limit;
  referee_info.shooter_barrel_cooling_value =
      game_robot_status.shooter_barrel_cooling_value;

  referee_info.shooter_id1_42mm_cooling_heat =
      power_heat_data.shooter_id1_42mm_cooling_heat;
}
