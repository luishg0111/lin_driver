/*
 * lin1d3_driver.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */
#include "lin1d3_driver.h"
#include <string.h>
#include <fsl_debug_console.h>

#define master_stack_size_d	(256)
#define master_task_priority (configMAX_PRIORITIES - 1)
#define master_queue_size_d	(8)

#define slave_stack_size_d	(256)
#define slave_task_priority (configMAX_PRIORITIES - 1)

#define size_of_uart_buffer	(10)

#define size_of_lin_header_d (2)

/*Static function prototypes */
static void master_task(void *pvParameters);
static void slave_task(void *pvParameters);



/******************************************************************************
 * Public functions
 *
 *****************************************************************************/

bool getBit(int num, int i)
{
    // Return true if the bit is
    // set. Otherwise return false
    return ((num & (1 << i)) != 0);
}

uint8_t calculate_parity(uint8_t header){
	uint8_t bit = 0;
	uint8_t parityEv = 0;
	uint8_t parityOd = 0;

	//Calculate Even parity bit
    for (int i = 7; i > 2; i--){
    	if(7 == i){
    		bit = getBit(header,  i);
    		parityEv = bit;

    	}else if (i !=4){     //skip bit 4
            bit = getBit(header,  i);
            parityEv = parityEv^bit;
        }
    }
    //Calculate Odd parity
    for (int i = 6; i > 1; i--){

    	if(6 == i){
    		bit = getBit(header,  i);
    		parityOd = bit;
    	}else if (i !=5){    //skip bit 5
            bit = getBit(header,  i);
            parityOd = parityOd^bit;
        }
    }
    //PRINTF("even: %d\r\n", parityEv);
    //PRINTF("odd: %d\r\n", parityOd);
    return (parityEv << 1)|parityOd;

}

#if defined(CALC_PARITY)
/*
 * Calculate parity bits
 * */
uint32_t lin1d3_calculateParity(uint8_t header)
{
	uint8_t lin1d4_p0 = 0x00;
	uint8_t lin1d4_p1 = 0x00;

	/* P0 = ID0 ^ ID1 ^ ID2 ^ ID4*/
	/* Even parity */
	lin1d4_p0 = ((header >> 7) & 1) ^ ((header >> 6) & 1) ^ ((header >> 5) & 1) ^ ((header >> 3) & 1);
	/* P1 = !(ID1 ^ ID3 ^ ID4 ^ ID5)*/
	/* Odd parity */
	lin1d4_p1 = !( ((header >> 6) & 1) ^ ((header >> 4) & 1) ^ ((header >> 3) & 1) ^ ((header >> 2) & 1) );

	/* Return parity into the header */
	return ((lin1d4_p0<<1) | lin1d4_p1);
}
#endif

#if defined(CALC_CHECKSUM)
/*
 * Calculates checksum based on Lin 1.3 Classic
 */
uint8_t lin1d3_checksum(uint8_t message[] , uint8_t len){
	uint16_t checksum = 0;

    for (size_t i = 0; i < len; i++) {
        checksum += message[i];
        checksum = (checksum & 0xFF) + (checksum >> 8); // Add carry if 8 bits overflow
    }

    return checksum;

	return (uint8_t)((~checksum) & 0xFF);
}
#endif

/*
 * Init a LIN node
 * */
lin1d3_handle_t* lin1d3_InitNode(lin1d3_nodeConfig_t config)
{
	lin1d3_handle_t* handle = NULL;
	static uint8_t node_idx = 0;
	char master_task_name[] = "linMaster0";
	char slave_task_name[] = "linSlave0";
	/* Check init parameters */
	if(config.type >= lin1d3_max_nodeType) {
		return NULL;
	}

	/* Create the handle structure and */
	handle = (lin1d3_handle_t*)pvPortMalloc(sizeof(lin1d3_handle_t));
	if(handle ==  NULL) {
		/* Failed to allocate memory for the node handle */
		return NULL;
	}
	/* Init the handle structure with 0s */
	memset(handle, 0, sizeof(lin1d3_handle_t));
	/* Copy the config */
	memcpy(&(handle->config), &config, sizeof(lin1d3_nodeConfig_t));

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		return NULL;
	}

	if(config.skip_uart_init == 0) {
		/* Create the handle UART handle structures */
		handle->uart_rtos_handle = (uart_rtos_handle_t*)pvPortMalloc(sizeof(uart_rtos_handle_t));
		if(handle->uart_rtos_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		handle->uart_handle = (uart_handle_t*)pvPortMalloc(sizeof(uart_handle_t));
		if(handle->uart_handle ==  NULL) {
			/* Failed to allocate memory for the node handle */
			return NULL;
		}

		if (0 > UART_RTOS_Init(handle->uart_rtos_handle, handle->uart_handle, &(handle->uart_config)))
		{
			return NULL;
		}
	}
	else {
		handle->uart_rtos_handle = config.uart_rtos_handle;
	}
	/* Create the Node Task */
	if(lin1d3_master_nodeType == config.type) {
		/* Create a queue for User message requests */
		handle->node_queue = xQueueCreate( master_queue_size_d, sizeof(uint8_t));
		if(handle->node_queue == NULL){
			vPortFree(handle);
			return NULL;
		}
		/* Create a task for the node */
		master_task_name[strlen(master_task_name)-1] += node_idx++;
		if (xTaskCreate(master_task, master_task_name, master_stack_size_d, handle, master_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}
	else if(lin1d3_slave_nodeType == config.type) {
		/* Create a task for the node */
		slave_task_name[strlen(slave_task_name)-1] += node_idx++;
		if (xTaskCreate(slave_task, slave_task_name, slave_stack_size_d, handle, slave_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}

	return handle;
}

/*
 * Send a message frame from a LIN Master node
 * */
uint32_t lin1d3_masterSendMessage(lin1d3_handle_t* handle, uint8_t ID)
{
	if(handle !=  NULL) {
		/* Put the requested ID on the master queue */
		xQueueSend( handle->node_queue, &ID, ( TickType_t ) 0 );
	}
	return 0;
}

/******************************************************************************
 * Static functions
 *
 *****************************************************************************/
static void master_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1d3_header[] = {0x55, 0x00};
	uint8_t  lin1d3_parity = 0;
	uint8_t  lin1d3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Wait for messages on the Queue */
        if(xQueueReceive(handle->node_queue, &ID, portMAX_DELAY)){
        	/* Build and send the LIN Header */
        	/* Put the ID into the header */
        	lin1d3_header[1] = ID<<2;

#if defined(CALC_PARITY)
        	/* Calculate parity bits */
        	lin1d3_parity = lin1d3_calculateParity(lin1d3_header[1]);

			/* Put the parity bits into the header */
			lin1d3_header[1] |= lin1d3_parity;
#endif

        	/* Init the message recevie buffer */
        	memset(lin1d3_message, 0, size_of_uart_buffer);
        	/* Calc the message size */
        	switch(ID&0x03) {
        		case 0x00: message_size = 2;
        		break;
        		case 0x01: message_size = 2;
        		break;
        		case 0x02: message_size = 4;
        		break;
        		case 0x03: message_size = 8;
        		break;
        	}
        	message_size+=1;

            /* Configure 13bit break transmission */
        	handle->uart_rtos_handle->base->S2 |= (1<<UART_S2_BRK13_SHIFT);

//            /* Configure 15bit synch break transmission */
//            handle->uart_rtos_handle->base->S2 |= (1<<2);
//            handle->uart_rtos_handle->base->BDH |= (1<<5);
//            handle->uart_rtos_handle->base->C1 &= (0<<4);

//        	/* Configure 16bit synch break transmission */
//        	handle->uart_rtos_handle->base->S2 |= (1<<2);
//        	handle->uart_rtos_handle->base->BDH |= (1<<5);
//        	handle->uart_rtos_handle->base->C1 |= (1<<4);

            /* Send the break signal */
        	handle->uart_rtos_handle->base->C2 |= UART_C2_SBK_MASK;
        	handle->uart_rtos_handle->base->C2 &= ~UART_C2_SBK_MASK;
        	vTaskDelay(1);

        	/* Send the header */
        	UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1d3_header, size_of_lin_header_d);
        	vTaskDelay(1);
        }
    }
}

static void slave_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1d3_header[size_of_lin_header_d];
	uint8_t  lin1d3_message[size_of_uart_buffer];
#if defined(CALC_PARITY)
	uint8_t  lin1d3_parity = 0;
#endif
#if defined(CALC_CHECKSUM)
	uint8_t  lin1d3_cksm;
#endif
	uint8_t  message_size = 0;
	size_t 	 n;
	uint8_t  msg_idx;


	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

    while(1) {
    	/* Init the message header buffer */
    	memset(lin1d3_header, 0, size_of_lin_header_d);

    	/* Wait for synch break */
    	//DisableIRQ(handle->config.irq); //Disable RX interrupt so the break won't mess with the UART_RTOS driver
    	handle->uart_rtos_handle->base->S2 |= 0x01<<UART_S2_LBKDIF_SHIFT; //Clear the LIN Break Detect Interrupt Flag
    	handle->uart_rtos_handle->base->S2 |= 0x01<<UART_S2_LBKDE_SHIFT; //Enable LIN Break Detection
    	while((handle->uart_rtos_handle->base->S2 &  0x01<<UART_S2_LBKDIF_SHIFT) == 0x00) vTaskDelay(1); //Wait for the flag to be set
    	handle->uart_rtos_handle->base->S2 &= ~(0x01<<UART_S2_LBKDE_SHIFT); //Disable LIN Break Detection
    	handle->uart_rtos_handle->base->S2 |= 0x01<<UART_S2_LBKDIF_SHIFT; //Clear the LIN Break Detect Interrupt Flag
    	//EnableIRQ(handle->config.irq); //Enable RX interrupt so the UART_RTOS driver works again

    	/* Wait for header on the UART */
    	UART_RTOS_Receive(handle->uart_rtos_handle, lin1d3_header, size_of_lin_header_d, &n);
    	/* Check header */
    	if((lin1d3_header[0] != 0x55))
    	{
    		/* Header is not correct we are ignoring the header */
    		continue;
    	}

#if defined(CALC_PARITY)
    	PRINTF("Header: %x \r\n" , lin1d3_header[1]);
    	/* Calculate parity bits */
    	lin1d3_parity = lin1d3_calculateParity(lin1d3_header[1]);

    	/* Check calculated parity bits are the same than the ones in header*/
    	if ((lin1d3_header[1] & lin1d3_parity) != lin1d3_parity)
    	{
    		PRINTF("Bad Parity \r\n");
    		/* ID parity bits are not correct we are ignoring message*/
    		continue;
    	}
#endif

    		/* Get the message ID */
			ID = (lin1d3_header[1] & 0xFC)>>2;
			/* If the header is correct, check if the message is in the table */
			msg_idx = 0;
			/*Look for the ID in the message table */
			while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
				if(handle->config.messageTable[msg_idx].ID == ID) {
					break;
				}
				msg_idx++;
			}
			/* If the message ID was not found then ignore it */
			if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;

			/* Calc the message size */
			switch(ID&0x03) {
				case 0x00: message_size = 2;
				break;
				case 0x01: message_size = 2;
				break;
				case 0x02: message_size = 4;
				break;
				case 0x03: message_size = 8;
				break;
			}

			message_size+=1;
			/* Init the message transmit buffer */
			memset(lin1d3_message, 0, size_of_uart_buffer);

			if(handle->config.messageTable[msg_idx].rx == 0) {
				/*If the message is in the table call the message callback */
				/* User shall fill the message */
				handle->config.messageTable[msg_idx].handler((void*)lin1d3_message);

#if defined(CALC_CHECKSUM)
				chsum = checksum(lin1d3_message , message_size);
				//PRINTF("chsum %x\n\r", chsum);
#endif
				/* Send the message data */
				UART_RTOS_Send(handle->uart_rtos_handle, (uint8_t *)lin1d3_message, message_size-1);
#if defined(CALC_CHECKSUM)
				UART_RTOS_Send(handle->uart_rtos_handle, &chsum, sizeof(chsum));
#endif
			}
			else {
				/* Wait for Response on the UART */
				UART_RTOS_Receive(handle->uart_rtos_handle, lin1d3_message, message_size, &n);
#if defined(CALC_CHECKSUM)
				chsum = checksum(lin1d3_message , n - 1);
				if(chsum != lin1d3_message[n-1])
				{
					continue;
				}
#endif
				/*If the message is in the table call the message callback */
				handle->config.messageTable[msg_idx].handler((void*)lin1d3_message);
			}

    	}

}
