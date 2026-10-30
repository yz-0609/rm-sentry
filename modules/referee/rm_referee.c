/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据
static uint8_t referee_rx_cache[RE_RX_BUFFER_SIZE * 2u]; // 保存跨回调的半包
static uint16_t referee_rx_used;

/* 已支持命令的数据段长度，未知命令返回0，仅校验并跳过整帧 */
static uint16_t JudgeDataLength(uint16_t cmd_id)
{
	switch (cmd_id)
	{
	case ID_game_state: return LEN_game_state;
	case ID_game_result: return LEN_game_result;
	case ID_game_robot_survivors: return LEN_game_robot_HP;
	case ID_event_data: return LEN_event_data;
	case ID_game_robot_state: return LEN_game_robot_state;
	case ID_power_heat_data: return LEN_power_heat_data;
	case ID_game_robot_pos: return LEN_game_robot_pos;
	case ID_buff_musk: return LEN_buff_musk;
	case ID_robot_hurt: return LEN_robot_hurt;
	case ID_shoot_data: return LEN_shoot_data;
	case ID_student_interactive: return LEN_receive_data;
	case ID_RFID_status: return LEN_RFID_status;
	case ID_ground_robot_pos: return LEN_ground_robot_pos;
	case ID_projectile_allowance: return LEN_projectile_allowance;
	case ID_sentry_decision: return LEN_sentry_decision;
	default: return 0;
	}
}

/**
 * @brief  缓存本次接收数据，循环解析完整帧，并保留未完成的半包
 * @param  buff 本次接收到的裁判系统原始数据
 * @param  recv_len 本次实际接收到的字节数
 */
static void JudgeReadData(uint8_t *buff, uint16_t recv_len)
{
	uint16_t offset = 0;

	if (buff == NULL || recv_len == 0 || recv_len > RE_RX_BUFFER_SIZE)
		return;

	if (recv_len > sizeof(referee_rx_cache) - referee_rx_used)
		referee_rx_used = 0; // 防止异常状态导致追加越界
	memcpy(referee_rx_cache + referee_rx_used, buff, recv_len);
	referee_rx_used += recv_len;

	while (referee_rx_used - offset >= LEN_HEADER)
	{
		uint8_t *frame = referee_rx_cache + offset;
		uint16_t data_length;
		uint16_t judge_length;
		uint16_t cmd_id;
		uint16_t expected_length;

		if (frame[SOF] != REFEREE_SOF || Verify_CRC8_Check_Sum(frame, LEN_HEADER) != TRUE)
		{
			++offset; // 杂字节或错误帧头，逐字节重新同步
			continue;
		}

		data_length = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
		// 先限制数据段长度，再做加法，避免异常长度越界或溢出
		if (data_length > RE_RX_BUFFER_SIZE - LEN_HEADER - LEN_CMDID - LEN_TAIL)
		{
			++offset;
			continue;
		}
		if (referee_rx_used - offset < DATA_Offset)
			break; // 命令码尚未收全

		cmd_id = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
		expected_length = JudgeDataLength(cmd_id);
		if (expected_length != 0 && data_length != expected_length)
		{
			++offset;
			continue;
		}

		judge_length = data_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
		if (referee_rx_used - offset < judge_length)
			break; // 半包保留到下一次接收
		if (Verify_CRC16_Check_Sum(frame, judge_length) != TRUE)
		{
			++offset;
			continue;
		}

		memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);
		referee_info.CmdID = cmd_id;
		switch (cmd_id)
		{
		case ID_game_state: // 0x0001 比赛状态数据
			memcpy(&referee_info.GameState, frame + DATA_Offset, LEN_game_state);
			break;
		case ID_game_result: // 0x0002 比赛结果数据
			memcpy(&referee_info.GameResult, frame + DATA_Offset, LEN_game_result);
			break;
		case ID_game_robot_survivors: // 0x0003 机器人血量数据
			memcpy(&referee_info.GameRobotHP, frame + DATA_Offset, LEN_game_robot_HP);
			break;
		case ID_event_data: // 0x0101 场地事件数据
			memcpy(&referee_info.EventData, frame + DATA_Offset, LEN_event_data);
			break;
		case ID_game_robot_state: // 0x0201 机器人性能体系数据
			memcpy(&referee_info.GameRobotState, frame + DATA_Offset, LEN_game_robot_state);
			break;
		case ID_power_heat_data: // 0x0202 缓冲能量和射击热量
			memcpy(&referee_info.PowerHeatData, frame + DATA_Offset, LEN_power_heat_data);
			break;
		case ID_game_robot_pos: // 0x0203 机器人位置数据
			memcpy(&referee_info.GameRobotPos, frame + DATA_Offset, LEN_game_robot_pos);
			break;
		case ID_buff_musk: // 0x0204 机器人增益数据
			memcpy(&referee_info.BuffMusk, frame + DATA_Offset, LEN_buff_musk);
			break;
		case ID_robot_hurt: // 0x0206 伤害状态数据
			memcpy(&referee_info.RobotHurt, frame + DATA_Offset, LEN_robot_hurt);
			break;
		case ID_shoot_data: // 0x0207 实时射击数据
			memcpy(&referee_info.ShootData, frame + DATA_Offset, LEN_shoot_data);
			break;
		case ID_student_interactive: // 0x0301 当前约定的5字节自定义内容
			memcpy(&referee_info.ReceiveData, frame + DATA_Offset, LEN_receive_data);
			break;
		case ID_RFID_status: // 0x0209 RFID状态数据
			memcpy(&referee_info.RFIDStatus, frame + DATA_Offset, LEN_RFID_status);
			break;
		case ID_ground_robot_pos: // 0x020B 地面机器人位置数据
			memcpy(&referee_info.GroundRobotPos, frame + DATA_Offset, LEN_ground_robot_pos);
			break;
		case ID_projectile_allowance: // 0x0208 允许发弹量数据
			memcpy(&referee_info.ProjectileAllowance, frame + DATA_Offset, LEN_projectile_allowance);
			break;
		case ID_sentry_decision: // 0x020D 哨兵自主决策信息同步
			memcpy(&referee_info.SentryInfo, frame + DATA_Offset, LEN_sentry_decision);
			break;
		}
		offset += judge_length;
	}

	if (offset != 0)
	{
		referee_rx_used -= offset;
		memmove(referee_rx_cache, referee_rx_cache + offset, referee_rx_used);
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->recv_len);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	referee_rx_used = 0;
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}



referee_info_t* GetRefereeInfo(void)
{
	return &referee_info;
}
