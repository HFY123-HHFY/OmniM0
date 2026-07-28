#include "Encoder.h"

/* API Encoder 层实现：维护编码器配置表，并分发到 G3507 Core 层。 */

/* 编码器速度值（EMA 滤波后，供 PID 速度环直接使用） */
int16_t Encoder1_Speed = 0;
int16_t Encoder2_Speed = 0;

#define API_ENCODER_MAX_ID  ((uint8_t)API_ENCODER_2)

/* ── EMA 低通滤波状态 ── */
static int32_t  s_filtered[API_ENCODER_MAX_ID + 1U];   /* Q16.16 内部累加器      */
static uint16_t s_filterAlpha[API_ENCODER_MAX_ID + 1U]; /* Q0.16 滤波强度         */
static uint8_t  s_stallCount[API_ENCODER_MAX_ID + 1U];  /* 连续 raw=0 计数，>=2→归零 */

static const API_Encoder_Config_t *s_encoderTable;
static uint8_t                      s_encoderCount;
static uint8_t                      s_encoderInited[API_ENCODER_MAX_ID + 1U];

static void API_Encoder_CoreInit(uint8_t coreId,
                                 void *portA, uint32_t pinA,
                                 void *portB, uint32_t pinB)
{
	G3507_Encoder_SetPins(coreId, portA, pinA, portB, pinB);
	G3507_Encoder_Init(coreId);
}

static int16_t API_Encoder_CoreGetCount(uint8_t coreId)
{
	return G3507_Encoder_GetCount(coreId);
}

static uint8_t API_Encoder_IsValidId(API_Encoder_Id_t id)


{
	return ((uint8_t)id <= API_ENCODER_MAX_ID) ? 1U : 0U;
}

static const API_Encoder_Config_t *API_Encoder_FindConfigById(API_Encoder_Id_t id)
{
	uint8_t i;

	if ((s_encoderTable == 0) || (s_encoderCount == 0U))
	{
		return 0;
	}

	for (i = 0U; i < s_encoderCount; ++i)
	{
		if (s_encoderTable[i].id == id)
		{
			return &s_encoderTable[i];
		}
	}

	return 0;
}

void API_Encoder_Register(const API_Encoder_Config_t *configTable, uint8_t count)
{
	s_encoderTable = configTable;
	s_encoderCount = count;
}

void API_Encoder_Init(API_Encoder_Id_t id)
{
	const API_Encoder_Config_t *config;

	if (API_Encoder_IsValidId(id) == 0U)
	{
		return;
	}

	config = API_Encoder_FindConfigById(id);
	if (config == 0)
	{
		return;
	}

	API_Encoder_CoreInit(config->coreId,
	                     config->portA, config->pinA,
	                     config->portB, config->pinB);

	s_encoderInited[(uint8_t)id] = 1U;

	/* 默认开启 EMA 滤波（α=0.3），可在 Init 后按需覆盖 */
	s_filterAlpha[(uint8_t)id] = ENCODER_FILTER_ALPHA_Q16;
	s_filtered[(uint8_t)id]   = 0;
	s_stallCount[(uint8_t)id]  = 0U;
}

int16_t API_Encoder_GetSpeed(API_Encoder_Id_t id)
{
	const API_Encoder_Config_t *config;

	if (API_Encoder_IsValidId(id) == 0U)
	{
		return 0;
	}

	if (s_encoderInited[(uint8_t)id] == 0U)
	{
		return 0;
	}

	config = API_Encoder_FindConfigById(id);
	if (config == 0)
	{
		return 0;
	}

	return API_Encoder_CoreGetCount(config->coreId);
}

/*
 * API_Encoder_SetFilterAlpha — 配置 EMA 滤波强度。
 *
 * @param alpha_q16  Q0.16 格式，0=无滤波，65535=最强（几乎冻结）
 *                   推荐：19661(≈0.3) 默认，32768(≈0.5) 轻滤波
 *                   调用时机：Init 之后、首次 GetFilteredSpeed 之前
 */
void API_Encoder_SetFilterAlpha(API_Encoder_Id_t id, uint16_t alpha_q16)
{
	if (API_Encoder_IsValidId(id) == 0U)
	{
		return;
	}

	s_filterAlpha[(uint8_t)id] = alpha_q16;

	/* 修改滤波系数后重置内部状态，防止旧值污染 */
	s_filtered[(uint8_t)id]  = 0;
	s_stallCount[(uint8_t)id] = 0U;
}

/*
 * API_Encoder_GetFilteredSpeed — 读取 EMA 滤波后的编码器速度。
 *
 * 堵转归零：
 *   连续 2 次 raw=0（40ms 无脉冲）→ 立即将滤波值归零。
 *   电机运行时 raw 交替 0/1 不会误触发（计数器在 raw≠0 时清零）。
 *
 * 内部逻辑：
 *   1) 读取当前 stable 快照（同 GetSpeed）
 *   2) 堵转检测：连续 2 次 raw=0 → 归零返回
 *   3) EMA：filtered += (raw - filtered) * alpha / 65536   （Q16.16 定点）
 *   4) 截断为 int16_t 返回
 *
 * 调用频率必须与 SnapshotAll 一致（20ms），否则滤波时间常数会偏移。
 * 首次调用用 raw 值直接初始化，无阶跃。
 */
int16_t API_Encoder_GetFilteredSpeed(API_Encoder_Id_t id)
{
	const API_Encoder_Config_t *config;
	int32_t raw, filtered;
	uint16_t alpha;
	uint8_t idx;

	if (API_Encoder_IsValidId(id) == 0U)
	{
		return 0;
	}

	idx = (uint8_t)id;
	if (s_encoderInited[idx] == 0U)
	{
		return 0;
	}

	config = API_Encoder_FindConfigById(id);
	if (config == 0)
	{
		return 0;
	}

	raw      = (int32_t)API_Encoder_CoreGetCount(config->coreId);
	alpha    = s_filterAlpha[idx];

	/* ── 堵转归零：连续 2 次 raw=0（40ms）→ 立即清零滤波值 ── */
	if (raw == 0)
	{
		if (++s_stallCount[idx] >= 2U)
		{
			s_filtered[idx] = 0;
			return 0;
		}
	}
	else
	{
		s_stallCount[idx] = 0U;
	}

	if (alpha == 0U)
	{
		/* 无滤波：直通 raw */
		s_filtered[idx] = raw << 16;
	}
	else
	{
		filtered = s_filtered[idx];

		if (filtered == 0)
		{
			/* 首次调用，直接初始化，避免从 0 缓慢爬升 */
			s_filtered[idx] = raw << 16;
		}
		else
		{
			/*
			 * EMA 定点更新（Q16.16）：
			 *   filtered = filtered + (raw - filtered) * alpha / 65536
			 * 展开：filtered = (filtered * (65536 - alpha) + raw * alpha) / 65536
			 * 统一乘除后用四舍五入防截断偏置。
			 */
			int32_t one_minus_alpha = (int32_t)(65536UL - (uint32_t)alpha);

			int64_t new_val = (int64_t)filtered * one_minus_alpha
			                + (int64_t)raw * 65536L * (int64_t)alpha;
			s_filtered[idx] = (int32_t)((new_val + 32768L) >> 16);
		}
	}

	/* 截断为 int16_t 返回 */
	filtered = s_filtered[idx] >> 16;
	if (filtered > 32767L)
	{
		return 32767;
	}
	if (filtered < -32768L)
	{
		return -32768;
	}
	return (int16_t)filtered;
}
