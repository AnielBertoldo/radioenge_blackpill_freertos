#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "radioenge_modem.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h> 

extern osTimerId_t PeriodicSendTimerHandle;
extern osThreadId_t AppSendTaskHandle;
extern ADC_HandleTypeDef hadc1;
extern osEventFlagsId_t ModemStatusFlagsHandle;
extern TIM_HandleTypeDef htim3;

typedef struct {
    uint32_t seq_no;
    int32_t  temp_oCx100;
} __attribute__((packed)) TEMPERATURE_OBJ_t;

typedef struct {
    uint16_t compressor_power;
    uint8_t  warning_status;
} __attribute__((packed)) AC_CMD_t;

osMessageQueueId_t acCmdQueueHandle;

void BlueLedTaskCode(void * argument)
{
    while (1)
    {   
        HAL_GPIO_TogglePin(LED4_BLUE_GPIO_Port, LED4_BLUE_Pin) ;
        osDelay(1000) ; 
    }
}

void GreenLedTaskCode(void * argument)
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED3_GREEN_GPIO_Port, LED3_GREEN_Pin) ;
        osDelay(500) ; 
    }
}

void YellowLedTaskCode(void * argument)
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin) ;
        osDelay(250) ; 
    }
}

void RedLedTaskCode(void * argument)
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED1_RED_GPIO_Port, LED1_RED_Pin) ;
        osDelay(100) ; 
    }
}

void LoRaWAN_RxEventCallback(uint8_t *data, uint32_t length, uint32_t port, int32_t rssi, int32_t snr)
{
    AC_CMD_t cmd;

    // Só aceita o pacote com o tamanho exato da struct (3 bytes)
    if (length != sizeof(AC_CMD_t))
        return;

    memcpy(&cmd, data, sizeof(AC_CMD_t));

    // timeout 0: o callback não pode bloquear
    osMessageQueuePut(acCmdQueueHandle, &cmd, 0, 0);
}

void ACControlTaskCode(void *argument)
{
    AC_CMD_t cmd;

    while (1)
    {
        if (osMessageQueueGet(acCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK)
        {
            uint32_t i = cmd.compressor_power;
            if (i > 100) i = 100;   // proteção

            htim3.Instance->CCR2 = (htim3.Instance->ARR * i) / 100;
        }
    }
}

// Chamar uma vez, antes do osKernelStart()
void AppACControlInit(void)
{
    acCmdQueueHandle = osMessageQueueNew(4, sizeof(AC_CMD_t), NULL);

    const osThreadAttr_t acControlAttr = {
        .name = "ACControl",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    osThreadNew(ACControlTaskCode, NULL, &acControlAttr);
}


void PeriodicSendTimerCallback(void *argument)
{
}



float DividerVoltageToDegreesCelsius(float Pin1Voltage_mV, float readVoltage_mV)
{
    const float cA = 0.001129148; // Steinhart-Hart Equation coefficient A
    const float cB = 0.000234125; // Steinhart-Hart Equation coefficient B
    const float cC = 0.0000000876741; // Steinhart-Hart Equation coefficient C
    const float divider_R = 10e3; // Value of the KY-013 fixed resistor (10k ohms)
    float thermistor_R, logThermistor_R; // Thermistor resistance and the natural logarithm of the thermistor resistance
    float tempThermistor;    

    // Calculate the thermistor resistance
    thermistor_R = (Pin1Voltage_mV * divider_R - divider_R * readVoltage_mV) / (readVoltage_mV);    
    // Calculate the natural logarithm of the thermistor resistance
    logThermistor_R = log(thermistor_R);    
    // Calculate the temperature in Kelvin using the Steinhart-Hart Equation
    tempThermistor = 1.0 / (cA + cB * logThermistor_R + cC * logThermistor_R * logThermistor_R * logThermistor_R);    
    // Convert Kelvin to Celsius and return
    
    return tempThermistor - 273.15;
}


void AppSendTaskCode(void *argument)
{
    uint32_t read;
    float temp;
    float readVoltage_mV;
    float Pin1Voltage_mV = 5000.0f;
    TEMPERATURE_OBJ_t payload;
    payload.seq_no = 0;

    while (1)
    {

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        read = HAL_ADC_GetValue(&hadc1);

        readVoltage_mV = 3300.0f * (float)read/4095.f;
        temp = DividerVoltageToDegreesCelsius(Pin1Voltage_mV, readVoltage_mV);

        payload.seq_no++;
        payload.temp_oCx100 = (int32_t)(temp * 100.0f);

        LoRaSendB(2, (uint8_t *)&payload, sizeof(TEMPERATURE_OBJ_t));
        osDelay(30000);
    }
}

