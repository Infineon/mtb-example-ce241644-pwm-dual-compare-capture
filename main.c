/*******************************************************************************
 * File Name:   main.c
 *
 * Description: The code example demonstrates the generation of asymmetric PWM
 * signals using two compare/capture registers available in TCPWM block of
 * Infineon devices. Compared to the asymmetric PWM realized with only one
 * compare function (where CPU is used to update the compare value two times in
 * every PWM cycle), this solution uses two independent buffered compare values
 * and generates less CPU load (where CPU is used to update the compare value once
 * in every PWM cycle). The CE talks about these advantages and its application
 * in the domain of field-oriented control applications.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 *******************************************************************************/
 
/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define COMPARE_VALUE_DELTA     (100)
#define DELAY_BETWEEN_READ_MS   (100)
#define W_KEY          0x77
#define A_KEY          0x61
#define S_KEY          0x73
#define D_KEY          0x64

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Debug UART variables */
static cy_stc_scb_uart_context_t    DEBUG_UART_context; /* UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj; /* Debug UART HAL object */

uint32_t period; /* Variable to store period value of TCPWM block */
int32_t compare0_value; /* Variable to store the CC0 value of TCPWM block */
int32_t compare1_value; /* Variable to store the CC1 value of TCPWM block */
int rec_cmd = 0; /* UART received command. */

/* Populate interrupt configuration structure */
cy_stc_sysint_t UART_SCB_IRQ_cfg =
{
    .intrSrc      = DEBUG_UART_IRQ,
    .intrPriority = 3u,
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void print_instructions(void);
void uart_event_handler(uint32_t event);
void Isr_uart1_fifo(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: Isr_uart1_fifo
********************************************************************************
* Summary:
* This function is registered to be called when UART1 interrupt occurs.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Isr_uart1_fifo(void)
{
    Cy_SCB_UART_Interrupt(DEBUG_UART_HW, &DEBUG_UART_context);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_scb_uart_status_t init_status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();


    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start UART operation */
    init_status = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (init_status!=CY_SCB_UART_SUCCESS)
    {
         CY_ASSERT(0);
    }

    /* Registers a callback function that notifies that
    *  uart_callback_events occurred in the Cy_SCB_UART_Interrupt.*/
    Cy_SCB_UART_RegisterCallback(DEBUG_UART_HW, (cy_cb_scb_uart_handle_events_t)uart_event_handler, &DEBUG_UART_context);

    /* Configuring priority and enabling NVIC IRQ
    * for the defined Service Request line number */
    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, Isr_uart1_fifo);
    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    /* Enable UART */
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config,
                                &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize redirecting of low level IO */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* retarget IO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_PPCA_Enable(CNFG_PPCA_INOUT_HW);

    Cy_PPCA_CNFG_PPCA_Output_Selector(CNFG_PPCA_INOUTCNFG_HW, &CNFG_PPCA_INOUT_ppcaOutConfig);

    /* Initialize and enable the TCPWM block */
     Cy_TCPWM_PWM_Init(PWM1_HW, PWM1_NUM,
               &PWM1_config);
     Cy_TCPWM_PWM_Enable(PWM1_HW, PWM1_NUM);

     /* Fetch the initial values of period, CC0 and CC1 registers configured
      * through the design file */
     period = Cy_TCPWM_PWM_GetPeriod0(PWM1_HW, PWM1_NUM);
     compare1_value = Cy_TCPWM_PWM_GetCompare1Val(PWM1_HW,
               PWM1_NUM);
     compare0_value = Cy_TCPWM_PWM_GetCompare0Val(PWM1_HW,
               PWM1_NUM);

     /* Start the TCPWM block */
     Cy_TCPWM_TriggerStart_Single(PWM1_HW, PWM1_NUM);

     /* Enable global interrupts */
     __enable_irq();

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************************************************************\r\n");
    printf("PSOC Control C3M/P8: TCPWM in PWM mode with dual compare/capture\r\n");
    printf("************************************************************\r\n\n");

    print_instructions();

    for (;;)
    {
        /* Delay between next read */
        Cy_SysLib_Delay(DELAY_BETWEEN_READ_MS);
    }
}

/********************************************************************************
 * Function Name: uart_event_handler
 ********************************************************************************
 * Summary:
 * Uart interrupt event handler callback function
 *
 * Parameters:
 *  handler_arg: user defined argument
 *  event: uart interrupt event source
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void uart_event_handler(uint32_t event)
{
    if (event == CY_SCB_UART_TRANSMIT_ERR_EVENT)
    {
        CY_ASSERT(0);
        /* An error occurred in Tx */
        /* Insert application code to handle Tx error */
    }
    else if (event == CY_SCB_UART_TRANSMIT_DONE_EVENT)
    {
        Cy_SCB_UART_ClearRingBuffer(DEBUG_UART_HW, &DEBUG_UART_context);
        /* All Tx data has been transmitted */
        /* Insert application code to handle Tx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_DONE_EVENT)
    {
        CY_ASSERT(0);
        /* All Rx data has been received */
        /* Insert application code to handle Rx done */
    }
    else if (event == CY_SCB_UART_RECEIVE_NOT_EMTPY)
    {
        /* Get input command */
        uint32_t read_value = Cy_SCB_UART_Get(DEBUG_UART_HW);

         printf("Pressed key: %c\r\n", (char)read_value);
        rec_cmd = (uint8_t)read_value;

        /* Distinguish command */
        switch(rec_cmd)
        {
        /* Increase duty cycle */
        case W_KEY:
            compare0_value += COMPARE_VALUE_DELTA;
            compare1_value += COMPARE_VALUE_DELTA;
            if( compare0_value > period )
                compare0_value = period;
            if( compare1_value > period )
                compare1_value = period;
            break;
            /* Decrease duty cycle */
        case S_KEY:
            compare0_value -= COMPARE_VALUE_DELTA;
            compare1_value -= COMPARE_VALUE_DELTA;
            if( compare0_value < 0 )
                compare0_value = 0;
            if( compare1_value < 0 )
                compare1_value = 0;
            break;
            /* Shift waveform to left */
        case A_KEY:
            compare0_value -= COMPARE_VALUE_DELTA;
            compare1_value += COMPARE_VALUE_DELTA;
            if( compare0_value < 0 )
                compare0_value = 0;
            if( compare1_value > period )
                compare1_value = period;
            break;
            /* Shift waveform to right */
        case D_KEY:
            compare0_value += COMPARE_VALUE_DELTA;
            compare1_value -= COMPARE_VALUE_DELTA;
            if( compare0_value > period )
                compare0_value = period;
            if( compare1_value < 0 )
                compare1_value = 0;
            break;
        default:
            printf("Wrong key pressed !! See below instructions:\r\n");
            print_instructions();
            return;
        }
        printf("Period: %lu\tCompare0: %ld\tCompare1: %ld\r\n",
                (unsigned long)period, (long)compare0_value,
                (long)compare1_value);

        /* Set new values for CC0/1 compare buffers */
        Cy_TCPWM_PWM_SetCompare0BufVal(PWM1_HW,
                PWM1_NUM, compare0_value);
        Cy_TCPWM_PWM_SetCompare1BufVal(PWM1_HW,
                PWM1_NUM, compare1_value);

        /* Trigger compare swap with its buffer values */
        Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM1_HW,
                PWM1_NUM);
    }
}

/*******************************************************************************
 * Function Name: print_instructions
 ********************************************************************************
 * Summary:
 * Prints set of instructions.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void print_instructions(void)
{
    printf("====================================================\r\n"
            "Instructions:\r\n"
            "====================================================\r\n"
            "Press 'w' : To increase the duty cycle\r\n"
            "Press 's' : To decrease the duty cycle\r\n"
            "Press 'a' : To shift waveform towards left\r\n"
            "Press 'd' : To shift waveform towards right\r\n"
            "====================================================\r\n");
}


/* [] END OF FILE */
