#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

EventGroupHandle_t xEventGroupSteps;
const EventBits_t bitStep1 = 0x01;
const EventBits_t bitStep2 = 0x02;

void vTaskStep1(void *pvParameters)
{
    for (;;)
    {
        printf("step1\n");
        xEventGroupSetBits(xEventGroupSteps, bitStep1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskStep2(void *pvParameters)
{
    float temperature = 0;
    float media = 0;
    for (;;)
    {
        printf("step2\n");
        xEventGroupSetBits(xEventGroupSteps, bitStep2);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void vTaskStep3(void *pvParameters)
{
    float temperature = 0;
    float media = 0;
    for (;;)
    {
        xEventGroupWaitBits(xEventGroupSteps, bitStep1 | bitStep2, 1, 1, portMAX_DELAY);
        media+=2;
        printf("media atual: %.2f\n", media);
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

int main(void)
{

    xEventGroupSteps = xEventGroupCreate();
    xTaskCreate(&vTaskStep1, "Task 1", 1024, NULL, 1, NULL);
    xTaskCreate(&vTaskStep2, "Task 2", 1024, NULL, 1, NULL);
    xTaskCreate(&vTaskStep3, "Task 3", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}


