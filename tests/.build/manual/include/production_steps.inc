static volatile int key_motion_request;
void AX_ROBOT_LatchFault(uint8_t code)
{
    taskENTER_CRITICAL();
    if (!ax_motion_fault) ax_motion_fault = code;
    ax_robot_move_enable = 0u;
    AX_ROBOT_Stop();
    taskEXIT_CRITICAL();
}
static void Key_Motion_Init(void)
{
    float speed = AX_DEMO_REQUEST_SPEED_MPS;
    float yaw = AX_TURN_MAX_YAW_RADPS;
    float track, wheel_yaw;
    float wheel_limit = AX_DEMO_OUTER_WHEEL_MPS;
#if ROBOT_TYPE == ROBOT_FWD
    track = (float)(FWD_WHEEL_BASE * FWD_WB_SCALE);
#elif ROBOT_TYPE == ROBOT_MEC
    track = (float)(MEC_WHEEL_BASE + MEC_ACLE_BASE);
#else
#error "Key motion requires FWD or MEC kinematics"
#endif
    if (speed > R_VX_LIMIT / 1000.0f) speed = R_VX_LIMIT / 1000.0f;
    if (speed > AX_TURN_MAX_FORWARD_MPS) speed = AX_TURN_MAX_FORWARD_MPS;
    if (wheel_limit > AX_TURN_MAX_WHEEL_MPS) wheel_limit = AX_TURN_MAX_WHEEL_MPS;
    /* Reserve steering range below the measured-speed drive cutoff. */
    if (speed > 0.90f * wheel_limit) speed = 0.90f * wheel_limit;
    if (yaw > R_VW_LIMIT / 1000.0f) yaw = R_VW_LIMIT / 1000.0f;
    wheel_yaw = 2.0f * (wheel_limit - speed) / track;
    if (yaw > wheel_yaw) yaw = wheel_yaw;
    if (yaw > 2.0f * speed / track) yaw = 2.0f * speed / track;
    AX_MOTION_Init(speed, yaw * 0.99f);
    key_motion_request = 0;
}
void Key_Motion_Step(void)
{
    int request;
    float gyro, strength;
    AX_TurnStatus status;
    const AX_MotionOutput *motion;
    taskENTER_CRITICAL();
    request = key_motion_request;
    key_motion_request = 0;
    taskEXIT_CRITICAL();
    /* Driver normalizes both supported IMUs to +/-500 degrees/second. */
    gyro = R_Imu.GYRO_Z * (500.0f / 32768.0f) * (3.141592654f / 180.0f)
           * AX_DEMO_GYRO_Z_SIGN;
    motion = AX_MOTION_Step(request, R_Vel.RT_IX / 1000.0f, gyro,
                            ax_control_dt_s,
                            ax_robot_move_enable && !ax_motion_fault &&
                            ax_control_mode == CTL_ROS);
    if (motion->fault) {
        AX_ROBOT_LatchFault(motion->fault);
        return;
    }
    if (!ax_robot_move_enable || ax_motion_fault || ax_control_mode != CTL_ROS) {
        AX_ROBOT_Stop();
        return;
    }
    strength = (float)fabs(motion->yaw_radps) * 100.0f / AX_TURN_MAX_YAW_RADPS;
    /* Always steer from the CURRENT ramp speed. Do not overwrite TG_IX with
     * AX_RAMP_Apply after this: steering may jointly limit speed and yaw.
     */
    status = motion->yaw_radps < 0.0f
        ? AX_TURN_Right(motion->speed_mps, strength)
        : AX_TURN_Left(motion->speed_mps, strength);
    if (status != AX_TURN_OK && status != AX_TURN_LIMITED) {
        AX_ROBOT_LatchFault(5u);
        return;
    }
    /* Also sample encoders while idle, so a new request sees fresh feedback. */
    AX_ROBOT_Kinematics();
}
static volatile uint8_t uart4_rx_ok;
static uint8_t uart4_rx_con;
static uint8_t uart4_rx_checksum;
static uint8_t uart4_rx_buf[AX_UART4_RX_DATA_MAX + 4u];
static uint8_t uart4_rx_ready[AX_UART4_RX_DATA_MAX];
volatile uint32_t ax_bluetooth_rx_bytes;
volatile uint32_t ax_bluetooth_rx_frames;
volatile uint8_t ax_bluetooth_last_byte;

void AX_BLUETOOTH_ReceiveByte(uint8_t byte)
{
    uint8_t i, count;
    ++ax_bluetooth_rx_bytes;
    ax_bluetooth_last_byte = byte;
    if (uart4_rx_con == 0u) {
        if (byte == 0xAAu) uart4_rx_con = 1u;
    } else if (uart4_rx_con == 1u) {
        if (byte == 0x55u) uart4_rx_con = 2u;
        else uart4_rx_con = byte == 0xAAu ? 1u : 0u;
    } else if (uart4_rx_con == 2u) {
        /* Bound both the ISR buffer and the caller's 16-byte buffer. */
        if (byte < 5u || byte > sizeof(uart4_rx_buf)) {
            uart4_rx_con = 0u;
        } else {
            uart4_rx_buf[2] = byte;
            uart4_rx_checksum = (uint8_t)(0xAAu + 0x55u + byte);
            uart4_rx_con = 3u;
        }
    } else if (uart4_rx_con < uart4_rx_buf[2] - 1u) {
        uart4_rx_buf[uart4_rx_con++] = byte;
        uart4_rx_checksum = (uint8_t)(uart4_rx_checksum + byte);
    } else {
        uart4_rx_con = 0u;
        if (byte == uart4_rx_checksum && uart4_rx_ok == 0u) {
            ++ax_bluetooth_rx_frames;
            count = uart4_rx_buf[2] - 4u;
            for (i = 0u; i < count; ++i) uart4_rx_ready[i] = uart4_rx_buf[3u + i];
            /* Publish only a complete frame; never overwrite an unread handshake. */
            uart4_rx_ok = count;
        }
    }
}
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == RESET) return;
    AX_BLUETOOTH_ReceiveByte((uint8_t)USART_ReceiveData(UART4));
}
void USART2_IRQHandler(void)
{
#if APP_BLUETOOTH_ENABLE && AX_BLUETOOTH_USE_USART2
    if (USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
        AX_BLUETOOTH_ReceiveByte((uint8_t)USART_ReceiveData(USART2));
#elif APP_KEY_TURN_TEST
    if (USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET) (void)USART_ReceiveData(USART2);
#else

	uint8_t Res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART2);	
		
		if(uart2_rx_con < 3)    //==接收帧头 + 长度
		{
			if(uart2_rx_con == 0)  //接收帧头1 0xAA
			{
				if(Res == 0xAA)
				{
					uart2_rx_buf[0] = Res;
					uart2_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart2_rx_con == 1) //接收帧头2 0x55
			{
				if(Res == 0x55)
				{
					uart2_rx_buf[1] = Res;
					uart2_rx_con = 2;
				}
				else
				{
					uart2_rx_con = 0;						
				}				
			}
			else  //接收数据长度
			{
				uart2_rx_buf[2] = Res;
				uart2_rx_con = 3;
				uart2_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
			}
		}
		else    //==接收数据
		{
			if(uart2_rx_con < (uart2_rx_buf[2]-1) )
			{
				uart2_rx_buf[uart2_rx_con] = Res;
				uart2_rx_con++;
				uart2_rx_checksum = uart2_rx_checksum + Res;					
			}
			else    //判断最后1位
			{
				//接收完成，恢复初始状态
				uart2_rx_con = 0;	
				
				//数据校验
				if( Res == uart2_rx_checksum )  //校验正确
				{	
					
					//设置为ROS控制模式
					ax_control_mode = CTL_ROS;
					
					//速度控制帧
					if(uart2_rx_buf[3] == ID_URX_VEL)
					{
						R_Vel.TG_IX = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
						R_Vel.TG_IY = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
						R_Vel.TG_IW = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
						
						
						//如果是阿克曼机器人，根据矢量速度计算前轮转向角
						#if (ROBOT_TYPE == ROBOT_AKM)
							ax_akm_angle = AX_AKM_WToAngle(R_Vel.TG_IX, R_Vel.TG_IW);
						#endif
					}
					else
					{
						//IMU陀螺仪校准
						if(uart2_rx_buf[3] == ID_URX_IMU)
						{
							ax_imu_calibrate_flag = uart2_rx_buf[4];
						}	

						//RGB灯效控制帧
						if(uart2_rx_buf[3] == ID_URX_LG)
						{
							R_Light.M  = uart2_rx_buf[4];
							R_Light.S  = uart2_rx_buf[5];
							R_Light.T  = uart2_rx_buf[6];
							R_Light.R  = uart2_rx_buf[7];
							R_Light.G  = uart2_rx_buf[8];
							R_Light.B  = uart2_rx_buf[9];
						}	

						//灯效EEPROM保存
						if(uart2_rx_buf[3] == ID_URX_LS)
						{
							//执行灯光效果保存动作
							ax_light_save_flag = 1;
						}	
						
						//蜂鸣器鸣叫控制
						if(uart2_rx_buf[3] == ID_URX_BP)
						{
							//设置为1蜂鸣器鸣叫
							if(uart2_rx_buf[4] != 0)
							{
								AX_BEEP_On();
							}
							else
							{
								AX_BEEP_Off();
							}
						}						
						
						//机器人底盘型号信息，用于判断是否一致
						if(uart2_rx_buf[3] == ID_URX_RTY)
						{
							//判断与当前底盘是否一致，
							if(uart2_rx_buf[4] == ROBOT_TYPE  )
							{
								//一致，则蜂鸣器短鸣叫提示
								ax_beep_ring = BEEP_SHORT;
							}
							else
							{
								//不一致，则蜂鸣器长鸣叫报警
								ax_beep_ring = BEEP_LONG;								
							}
						}						
					}		
				}
			}
		}
		
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	} 

#endif
}
uint8_t AX_UART4_GetData(uint8_t *pbuf)
{
    uint8_t count, i;
    /* Mask this IRQ explicitly: its priority is above the RTOS critical mask. */
#if AX_BLUETOOTH_USE_USART2
    NVIC_DisableIRQ(USART2_IRQn);
#else
    NVIC_DisableIRQ(UART4_IRQn);
#endif
    count = uart4_rx_ok;
    for (i = 0u; i < count; ++i) pbuf[i] = uart4_rx_ready[i];
    uart4_rx_ok = 0u;
#if AX_BLUETOOTH_USE_USART2
    NVIC_EnableIRQ(USART2_IRQn);
#else
    NVIC_EnableIRQ(UART4_IRQn);
#endif
    return count;
}
void AX_CTL_App(void)
{
    static TickType_t last_motion_tick;
    static uint8_t motion_valid;
    uint8_t comdata[AX_UART4_RX_DATA_MAX];
    uint8_t count = AX_UART4_GetData(comdata);
    TickType_t now = xTaskGetTickCount();
#if ROBOT_TYPE != ROBOT_AKM
    int32_t tilt;
#endif

    if (count >= 2u && comdata[0] == ID_BLERX_CM && comdata[1] == 0x55u) {
        /* Transfer ownership in the control task, never in the serial ISR. */
        AX_ROBOT_Stop();
        motion_valid = 0u;
        ax_control_mode = CTL_APP;
        ax_beep_ring = BEEP_SHORT;
    }
    if (ax_control_mode != CTL_APP) {
        motion_valid = 0u;
        return;
    }
    if (ax_robot_move_enable && !ax_motion_fault) {
        if (count >= 5u && (comdata[0] == ID_BLERX_YG || comdata[0] == ID_BLERX_SB)) {
            R_Vel.TG_IX = (int16_t)(6 * (int8_t)comdata[4]);
            R_Vel.TG_IY = (int16_t)(-6 * (int8_t)comdata[3]);
#if ROBOT_TYPE == ROBOT_AKM
            ax_akm_angle = (int16_t)(-5 * (int8_t)comdata[1]);
#else
            R_Vel.TG_IW = (int16_t)(-20 * (int8_t)comdata[1]);
#endif
            motion_valid = 1u;
            last_motion_tick = now;
        } else if (count >= 4u && comdata[0] == ID_BLERX_ZL) {
            R_Vel.TG_IX = (int16_t)(-6 * (int8_t)comdata[3]);
            R_Vel.TG_IY = 0;
#if ROBOT_TYPE == ROBOT_AKM
            ax_akm_angle = (int16_t)(-6 * (int8_t)comdata[2]);
#else
            /* Roll changes yaw, which FWD kinematics turns into wheel difference.
             * Use the entire APP angle range instead of saturating at +/-20.
             */
            tilt = (int8_t)comdata[2];
            if (tilt > APP_GRAVITY_STEER_FULL_SCALE) tilt = APP_GRAVITY_STEER_FULL_SCALE;
            if (tilt < -APP_GRAVITY_STEER_FULL_SCALE) tilt = -APP_GRAVITY_STEER_FULL_SCALE;
            R_Vel.TG_IW = (int16_t)(-tilt * R_VW_LIMIT / APP_GRAVITY_STEER_FULL_SCALE);
#endif
            motion_valid = 1u;
            last_motion_tick = now;
        }
    }
    if (count >= 7u && comdata[0] == ID_BLERX_LG) {
        R_Light.M = comdata[1]; R_Light.S = comdata[2]; R_Light.T = comdata[3];
        R_Light.R = comdata[4]; R_Light.G = comdata[5]; R_Light.B = comdata[6];
    } else if (count >= 2u && comdata[0] == ID_BLERX_LS && comdata[1] == 0x55u) {
        ax_light_save_flag = 1u;
    }
    /* Lights, handshake, noise and short packets do not renew the motion lease. */
    if (!ax_robot_move_enable || ax_motion_fault || !motion_valid ||
        (TickType_t)(now - last_motion_tick) >= pdMS_TO_TICKS(APP_BLUETOOTH_TIMEOUT_MS)) {
        motion_valid = 0u;
        AX_ROBOT_Stop();
    }
}
void Robot_Task(void* parameter)
{	

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime;
    TickType_t last_cycle,now,gap;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime = xTaskGetTickCount();
    last_cycle=PreviousWakeTime;
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement );
        now=xTaskGetTickCount(); gap=now-last_cycle; last_cycle=now;
        if (gap==0u || gap>pdMS_TO_TICKS(AX_CONTROL_MAX_GAP_MS)) {
            AX_ROBOT_LatchFault(2u);
        }
        ax_control_dt_s=(gap ? gap : 1u)/(float)configTICK_RATE_HZ;
        ROBOT_IMUHandle();
        if (!AX_MPU6050_IsReady()) { AX_ROBOT_LatchFault(1u); }

		
#if APP_BLUETOOTH_ENABLE
        /* Poll the connection frame even while keys own the chassis. */
        AX_CTL_App();
#endif
#if APP_KEY_TURN_TEST
        if (ax_control_mode == CTL_APP) {
            if (ax_robot_move_enable && !ax_motion_fault) AX_ROBOT_Kinematics();
            else AX_ROBOT_Stop();
        } else {
            Key_Motion_Step();
        }
#else
        if (ax_motion_fault) ax_robot_move_enable = 0u;

		//控制方式选择
		if(ax_control_mode != 0)
		{
			if      (ax_control_mode == CTL_PS2)    AX_CTL_Ps2();    //PS2手柄控制
#if !APP_BLUETOOTH_ENABLE
			else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP控制
#endif
			else if (ax_control_mode == CTL_RMS)    AX_CTL_RemoteSbus();   //SBUS航模遥控器控制
		}
		
		//判断机器人运动是否开启
		if(ax_robot_move_enable != 0)
		{
			//机器人运动学处理
			AX_ROBOT_Kinematics();				
		}
		else
		{
			//机器人静止不动
			AX_ROBOT_Stop();				
		}
		
#endif


		
		//数据发送
		ROBOT_SendDataToRos();
	}
}
static int16_t AX_StartupGyroOffset(int32_t sum)
{
    int32_t offset = -(sum / 10);
    if (offset > 32767) offset = 32767;
    if (offset < -32768) offset = -32768;
    return (int16_t)offset;
}