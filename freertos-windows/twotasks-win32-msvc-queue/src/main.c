#include "stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

xQueueHandle xTemperatureQueue;

void vTaskReadTemperature(void *pvParameters)
{

    float temperature = 0;

    for (;;)
    {
        temperature = 20 + (rand()/(RAND_MAX/10.0));
        BaseType_t isSuccessful = xQueueSend(xTemperatureQueue, &temperature, pdMS_TO_TICKS(1000));
        if(isSuccessful) {
            printf("temperatura enviada: %.2f\n", temperature);
        } else {
            printf("deu pau\n");            
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vTaskCalculate(void *pvParameters)
{
    float temperature = 0;
    float media = 0;
    for (;;)
    {
        if(xQueueReceive(xTemperatureQueue, &temperature, pdMS_TO_TICKS(2000))){
            media = (media+temperature)/2.0;
        } else {
            printf("deu ruim\n");            
        }
        printf("media atual: %.2f\n", media);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


int main(void)
{

    xTemperatureQueue = xQueueCreate(5, sizeof(float));

    xTaskCreate(&vTaskReadTemperature, "Task 1", 1024, NULL, 1, NULL);
    xTaskCreate(&vTaskCalculate, "Task 2", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}


