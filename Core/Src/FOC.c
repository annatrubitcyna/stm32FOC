#include "FOC.h"

extern TIM_HandleTypeDef htim4;
extern CORDIC_HandleTypeDef hcordic;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef status;


float normalizeAngle(float angle)  //[0, 2pi]
{
	float a = fmodf(angle, PI_2);
    return a >= 0 ? a : (a + PI_2);
}
float normalizeAnglePI(float angle)  //[-pi, pi]
{
	float a = fmodf(angle, PI_2);
	if( a > M_PI){
		a -= PI_2;
	}
	if( a < -M_PI){
		a += PI_2;
	}
    return a;
}

float saturate(float a, float amin, float amax)
{
	if ( a > amax ){
		return amax;
	}
	else if (a < amin){
		return amin;
	}
	return a;
}

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#define q31_to_f32(x) ldexp((int32_t) x, -31)

static inline int f32_to_q31(float input)
{
	const float Q31_MAX_F = 0x0.FFFFFFp0F;
	const float Q31_MIN_F = -1.0F;
	return (int)roundf(scalbnf(fmaxf(fminf(input, Q31_MAX_F), Q31_MIN_F), 31));
}

////////////////////////////   UART   ////////////////////////////

void sendByUart(struct Motor* m, struct SentData* sentData)
{
	sentData->refAngle = m->refAngle;
	sentData->angle =  m->mechAngle;
	sentData->refSpeed = m->refSpeed;
	sentData->speed = m->speed;
 	sentData->Id = m->IqRef;
	sentData->Iq = m->Iq;
	sentData->time = m->time;
	sentData->Uq = m->Uq;

	status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)sentData, sizeof(*sentData));
}

void getByUart(struct Motor* m, struct RefData* refData)
{
	status = HAL_UART_Receive_DMA(&huart1, (uint8_t*)refData, 20);
	if(status ==  HAL_OK){
		m->sC.Kp = refData->Kp;
		m->sC.Ki = refData->Ki;
		m->sC.Kd = refData->Kd;
	}
}
////////////////////////////   button   ////////////////////////////

void changeEn(struct Motor* m)
{
	if(fabs(m->speed) < 0.1){
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1); //run
		m->eISpeed = 0;
	}
	else{
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0); //stop
	}
}

void stopRun(struct Motor* m){
	if(m->refSpeed !=0){
		m->refSpeed = 0 * PI_2 / 60;
	}
	else{
		m->refSpeed = m->startRefSpeed;
	}
}

void changeSpeed(struct Motor* m)
{
	m->eISpeed = 0;
	if(m->refSpeed < -20){
		m->speedSign = 1;
	}
	if(m->refSpeed > 20){
		m->speedSign = -1;
	}
	m->refSpeed += m->speedSign * 20 * PI_2 / 60;
}

void changeAngle(struct Motor* m)
{
	m->refAngle += 20 *M_PI/180;
}


////////////////////////////   CORDIC   ////////////////////////////

float cordic_q31_sinf(float x, CORDIC_HandleTypeDef* hcordic)
{
	int32_t input_q31 = f32_to_q31(x /(M_PI));    // from -PI to PI
	int32_t output_q31[2] = {0};

	hcordic->Instance->CSR &= 0xFFFFFFF0; // function cosine
	hcordic->Instance->WDATA = input_q31;
	hcordic->Instance->WDATA = f32_to_q31(1); // to use 2 arguments in all functions (scale = 1)
	output_q31[0] = hcordic->Instance->RDATA;
	output_q31[1] = hcordic->Instance->RDATA;
	float ans = (float)q31_to_f32(output_q31[1]);
	return ans;
}

static inline void cordic_q31_cos_sinf(struct trAngle* x, CORDIC_HandleTypeDef* hcordic)
{
	int32_t input_q31 = f32_to_q31(normalizeAnglePI(x->an) / M_PI); // [0, 2*PI] -> [-1, 1]
	int32_t output_q31[2] = {0};

	hcordic->Instance->CSR &= 0xFFFFFFF0; // function cosine
	hcordic->Instance->WDATA = input_q31;
	hcordic->Instance->WDATA = f32_to_q31(1);
	output_q31[0] = hcordic->Instance->RDATA;
	output_q31[1] = hcordic->Instance->RDATA;
	x->cos = (float)q31_to_f32(output_q31[0]);
	x->sin = (float)q31_to_f32(output_q31[1]);
}

static inline void cordic_q31_atanf(float x, float y, CORDIC_HandleTypeDef* hcordic, struct Motor* m)
{
	float maxMod = 3 * m->Umax; // more than sqrt(2) to made answer less than 1
	int32_t input_q31[2] = {0};
	input_q31[0] = f32_to_q31(x / maxMod);
	input_q31[1] = f32_to_q31(y / maxMod);
	int32_t output_q31[2] = {0};

	hcordic->Instance->CSR &= 0xFFFFFFF0;
	hcordic->Instance->CSR |= 0x00000002; // function phase

	hcordic->Instance->WDATA = input_q31[0];
	hcordic->Instance->WDATA = input_q31[1];
	output_q31[0] = hcordic->Instance->RDATA; // from -1 to 1: -PI to PI
	output_q31[1] = hcordic->Instance->RDATA; // from 0 to 1

	m->alpha = normalizeAngle((float)q31_to_f32(output_q31[0]) * M_PI);
	m->U = (float)q31_to_f32(output_q31[1]) * maxMod;
}

////////////////////////////   vector PWM   ////////////////////////////

uint32_t getSector(float alpha)
{
	return alpha / PI_1_3;
}

float* getPeriodsABC(int sector, float t1, float t2, float t3, float T0_1_2)
{
	static float Periods[3] = {0};
	switch( sector )
	{
	    case 0:
	    	Periods[0] = t1;
	    	Periods[1] = t2;
	    	Periods[2] = T0_1_2;
	    	return Periods;
	    case 1:
			Periods[0] = t3;
			Periods[1] = t1;
			Periods[2] = T0_1_2;
			return Periods;
	    case 2:
			Periods[0] = T0_1_2;
			Periods[1] = t1;
			Periods[2] = t2;
			return Periods;
	    case 3:
			Periods[0] = T0_1_2;
			Periods[1] = t3;
			Periods[2] = t1;
			return Periods;
	    case 4:
			Periods[0] = t2;
			Periods[1] = T0_1_2;
			Periods[2] = t1;
			return Periods;
	    case 5:
			Periods[0] = t1;
			Periods[1] = T0_1_2;
			Periods[2] = t3;
			return Periods;
	}
	return Periods;
}


float* getPeriods(struct Motor* m)
{
	uint32_t sector = getSector(m->alpha);
	float betta = m->alpha - sector * PI_1_3; // angle inside sector
	float Tb1 = m->U * cordic_q31_sinf( PI_1_3 - betta , m->hcordic);  // or arm_sin_f32
	float Tb2 = m->U * cordic_q31_sinf( betta, m->hcordic);

	float T0 = 1 - Tb1 - Tb2;
	float T0_1_2 = T0 * 0.5;
	float t1 = Tb1 + Tb2 + T0_1_2;
	float t2 = Tb2 + T0_1_2;
	float t3 = Tb1 + T0_1_2;
	float* periods = getPeriodsABC(sector, t1, t2, t3, T0_1_2);
	return periods;
}

////////////////////////////   encoder   ////////////////////////////

void processEncoderData(struct Motor* m)
{
	m->data = m->data & 0x3FFF;  // to get only 14 last bits
	m->mechAngle = normalizeAngle(m->data * encConst);

	m->elAngle.an = normalizeAngle (m->mechAngle * m->zp - m->startAngle); // zero shift + number of pole pairs
	cordic_q31_cos_sinf(&(m->elAngle), m->hcordic);

	// count turns
	if (m->mechAngle <= PI_1_2){
		if (PI_3_2 <= m->pMAngle){
			m->kTurns += 1;  // moved from forth quarter to first
		}
	}
	else if (PI_3_2 <= m->mechAngle){
		if (m->pMAngle <= PI_1_2){
			m->kTurns -= 1;  // moved from first quarter to forth
		}
	}
	m->pMAngle = m->mechAngle; //to count turns
}

uint32_t checkParity(uint16_t data)
{
	uint32_t ans = 0;
	for(int i=0; i<15; i++){
		uint16_t x = (data >> i) & 0x1;
		ans+= x;
	}
	if((data >> 15) == (ans & 0x1)){
		return 1;
	}
	else{
		return 0;
	}
}

void updateEncData(struct Motor* m)
{
	uint16_t address = (uint16_t)(ANGLECOM);
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&address, (uint8_t *)&(m->data), 1, 100);
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	if(checkParity(m->data)){
		processEncoderData(m);
	}
}

////////////////////////////   angle controllers   ////////////////////////////

void seriesCompensator(struct Motor* m)
{
	float mu = 80;
	float sigma_k = 400;
	float e =  normalizeAnglePI(m->refAngle - m->mechAngle);
	m->eIAngle += e;
	float Ki = 0.01;
	float In =  (mu * (e - m->speed) - m->IqRef * m->speedFrequency) / (m->speedFrequency + sigma_k) + Ki * m->eIAngle;
	m->IqRef = saturate (In, -m->Imax, m->Imax);
}

void angleImpedanceController(struct Motor* m)
{
    float f = 1000000;    //speed
    float zeta = 0;       //vibration
    float r = 100;        //reaction
    float e =  normalizeAnglePI(m->refAngle - m->mechAngle);
    float k1  = zeta / (M_PI * f);
    float k2  = 1 / pow(PI_2 * f, 2);
    float k3 = r * zeta / (PI_2 *  f);

    float speedN =( e - k1 * m->speed - k2 * m->dSpeed / m->speedFrequency + m->refSpeed * m->speedFrequency * k3)/(1 + k3 * m->speedFrequency ) ;
    m->refSpeed = saturate (speedN, -m->speedMax, m->speedMax);
}

void angleController(struct Motor* m)
{
	//idling 5.97, 0, 30
    float Kp = 2.97;
    float Ki = 0.01;
    float Kd = 1450;
    float e =  normalizeAnglePI(m->refAngle - m->mechAngle);
    m->eIAngle += e;
    m->prevMAngle = m->mechAngle;
    float speedN = Kp * e + Ki * m->eIAngle - Kd * normalizeAngle(m->mechAngle - m->prevMAngle - m->kTurns * PI_2)  * m->speedFrequency;
    m->refSpeed = saturate (speedN, -m->speedMax, m->speedMax);
}

////////////////////////////   speed   ////////////////////////////

float observer(struct Motor* m)
{
	struct Observer* o = &(m->obs);
	o->x1_hat = (o->x2_hat - o->l1*(o->y_hat - (m->mechAngle + m->kTurns * PI_2))) / m->speedFrequency + o->x1_hat;
	m->dSpeed = (m->Iq - o->l2*(o->y_hat - (m->mechAngle + m->kTurns * PI_2))) / m->speedFrequency;
	o->x2_hat =  m->dSpeed + o->x2_hat;
	o->y_hat = o->x1_hat;
	return o->x2_hat;
}

void countSpeed(struct Motor* m)
{
	m->time+= (float)DWT->CYCCNT / SystemCoreClock;
	if(!m->flagSpeed) {
		m->speedFrequency = SystemCoreClock / DWT->CYCCNT;
		m->speed = observer(m);
	}
	else {
		m->speed = 0;
		m->flagSpeed = 0;
	}
	DWT->CYCCNT = 0U;
}

void speedController(struct Motor* m)
{
    float e = m->refSpeed - m->speed;
    m->eISpeed += e;
    float In = m->sC.Kp * e + m->sC.Ki * m->eISpeed - m->sC.Kd * m->dSpeed / m->speedFrequency;
    m->IqRef = saturate (In, -m->Imax, m->Imax);
}

////////////////////////////   current   ////////////////////////////

float med(float x[3], float I)
{
	for (int i =0 ; i < 2 ; i++){
		x[i] =x[i+1];
	}
	x[2] = I;
	if(x[0] < x[2]){
		if (x[1] < x[0]){
			return x[0];
		}
		else if (x[1]> x[2]){
			return x[2];
		}
		else{
			return x[1];
		}
	}
	else{
		if (x[1] < x[2]){
			return x[2];
		}
		else if (x[1]> x[0]){
			return x[0];
		}
		else{
			return x[1];
		}
	}
}

float fast_med(float arr[10], float I, uint32_t n)
{
	for (int i =0 ; i < n-1 ; i++){
		arr[i] =arr[i+1];
	}
	arr[n-1] = I;
	uint16_t low, high ;
	uint16_t median;
	uint16_t middle, ll, hh;
    low = 0 ; high = n-1 ; median = (low + high) / 2;
    for (;;) {
		if (high <= low) /* One element only */
			return arr[median] ;
		if (high == low + 1) { /* Two elements only */
			if (arr[low] > arr[high])
				ELEM_SWAP(arr[low], arr[high]) ;
				return arr[median] ;
			}
		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (arr[middle] > arr[high])
			ELEM_SWAP(arr[middle], arr[high]) ;
		if (arr[low] > arr[high])
			ELEM_SWAP(arr[low], arr[high]) ;
		if (arr[middle] > arr[low])
			ELEM_SWAP(arr[middle], arr[low]) ;
		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(arr[middle], arr[low+1]) ;
		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (arr[low] > arr[ll]) ;
			do hh--; while (arr[hh] > arr[low]) ;
			if (hh < ll)
				break;
			ELEM_SWAP(arr[ll], arr[hh]) ;
		}
		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(arr[low], arr[hh]) ;
		/* Re-set active partition */
		if (hh <= median)
			low = ll;
		if (hh >= median)
			high = hh - 1;
    }
    return arr[median] ;
}

float cur_filter(struct Motor* m, int i, float I)
{
	struct Filter* f = &(m->filter);
	f->x_med[i] = med(f->x[i], I);

	float dx = f->x_med[i] - f->x_hat[i];
	if (fabs(dx) > f->dx_max){
		f->x_hat[i] += dx * f->k_max ;
	}
	else{
		f->x_hat[i] += dx * f->k;
	}
	return f->x_hat[i];
}

void getCurrents(struct Motor* m)
{
	m->IA = (m->adcData[0] - 2048) * curSenseConst + 0.1;
	m->IA = cur_filter(m, 0, m->IA);
	m->IB = -(m->adcData[1] - 2048) * curSenseConst - 0.1;
	m->IB = cur_filter(m, 1, m->IB);
}

void countCurrents(struct Motor* m)
{
	float Ialpha = m->IA;
	float Ibetta = D_SQRT3 * m->IA + D2_SQRT3 * m->IB;

	m->Id = Ibetta * m->elAngle.sin + Ialpha * m->elAngle.cos;
	m->Iq = Ibetta * m->elAngle.cos - Ialpha * m->elAngle.sin;
}

void currentController(struct Motor* m)
{
	// idling Kpd = 0.6, Kid = 0.00001, Kpq = 0.5, Kiq = 0
	// with weight 2 Kpq = 0.7, Kiq = 0.0005
	float Kpd = 0.6;
	float Kid = 0.00001;
	float Kpq = 0.7;   // L/Tt = 0.0138/0.005 = 2.76
	float Kiq = 0.0005; // R/Tt = 9.9/0.005 = 1980

	float eq = m->IqRef - m->Iq;
	float ed = m->IdRef - m->Id;

	m->eIq += eq;
	m->eId += ed;
    float Unq = Kpq * eq + Kiq * m->eIq;
    float Und = Kpd * ed + Kid * m->eId;
    m->Uq = saturate (Unq, -m->Umax, m->Umax);
    m->Ud = saturate (Und, -m->Umax, m->Umax);
}

void limitingCurrentController(struct Motor* m)
{
	float Kpd = 0.6;
	float Kid = 0.00001;
	float Unq;
	if(m->Iq > m->Imax){
		Unq = 0;
	}
	else {
		Unq = m->IqRef;
	}

	float ed = m->IdRef - m->Id;
	m->eId += ed;
    float Und = Kpd * ed + Kid * m->eId;
    m->Uq = saturate (Unq, -m->Umax, m->Umax);
    m->Ud = saturate (Und, -m->Umax, m->Umax);
}

////////////////////////////   invCoordsTransform   ////////////////////////////

void invCoordsTransform(struct Motor* m)
{
	float Ualpha = m->Ud * m->elAngle.cos - m->Uq * m->elAngle.sin;
	float Ubetta = m->Ud * m->elAngle.sin + m->Uq * m->elAngle.cos;
	cordic_q31_atanf(Ualpha, Ubetta, m->hcordic, m);
}

void setPeriods(struct Motor* m)
{
	float* periods = getPeriods(m);
	TIM1->CCR2 = TIM1->ARR * periods[0];
	TIM1->CCR3 = TIM1->ARR * periods[1];
	TIM1->CCR4 = TIM1->ARR * periods[2];
}

////////////////////////////   initialization   ////////////////////////////

void InitEncoder(struct Motor* m){
	m->U = m->Umax;
	m->alpha = 0;
	setPeriods(m);
	HAL_Delay(1000);
	updateEncData(m); //sent request to read data
	updateEncData(m); //read data
	m->startAngle = m->elAngle.an;
}
void InitCordic(CORDIC_HandleTypeDef* hcordic){
	//xxxx xxxx x001 1000(no int) xxxx x000 0110 (6cicles) 0000 cosine(0001 -sine, 0010 - phase)
	hcordic->Instance->CSR |= 0x00180060 ; //0000 0000 0001 1000 0000 0000 0110 0000
	hcordic->Instance->CSR &= 0xFFF9F860 ; //1111 1111 1001 1000 1111 1000 0110 0000
}
void InitFilter(struct Filter* f){
	for(int i =0; i<4; i++){
		f->x_med[i] = 0;
		f->x_hat[i] = 0;
		for(int j =0; j<3 ; j++){
			f->x[i][j] = 0;
		}
	}
	f->k = 0.1; //if more new value has bigger part
	f->dx_max = 0.5; // if current change for that value k change to k_max
	f->k_max = 0.9;
}
void InitObserver(struct Observer* o){
	o->y_hat = 0;
	o->x1_hat = 0;
	o->x2_hat = 0;
	o->q1 = 0;
	o->q2 = 10;
	o->r = 0.001;
	arm_sqrt_f32 (o->q2 / o->r , &(o->l2));
	arm_sqrt_f32 (2 * o->l2 + o->q1 / o->r , &(o->l1));
}

__STATIC_INLINE void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}
void InitMotor(struct Motor* m,CORDIC_HandleTypeDef* hcordic){
	m->zp = 14;
	m->R = 9.9;
	m->startAngle = 0;
	m->isEncInit = 0;
	m->data = 0;
	m->startRefSpeed = -100 * PI_2 / 60;   // rpm -> rad/sec
	m->refSpeed = 0 ;                      // rad/sec
	m->refAngle = 1 * M_PI / 180;          //radians
	m->Umax = 1;
	m->Imax = 4;
	m->speedMax = 200 * PI_2 / 60;
	m->IdRef = 0;
	m->hcordic = hcordic;
	m->speedPerCounter = 0; //for first loop
	m->speedFrequency = 100; //frequency = 20000/ 200
	m->flagSpeed = 1;
	//idling - Kp = 0.6, Ki = 0.005, Kd = 500.
	//with weight1 - 2.2, 0.0, 1600 *3
	//with weight2 - 0.9, 0.0005, 1970
    m->sC.Kp = 0.9;
    m->sC.Ki = 0.0005;
    m->sC.Kd = 19700;
	InitCordic(hcordic);
	InitFilter(&(m->filter));
	InitObserver(&(m->obs));
	InitEncoder(m);
	DWT_Init();
	m->time = 0;
	m->speedSign = 1;
}

void mainLoop(struct Motor* m)
{
	updateEncData(m);
	getCurrents(m);
	countCurrents(m);
	currentController(m);
	if(m->speedPerCounter > 20){
		countSpeed(m);
		speedController(m);
		angleController(m);
		m->speedPerCounter = 0;
	}
	else{
		m->speedPerCounter +=1;
	}
	invCoordsTransform(m);
	setPeriods(m);
}

