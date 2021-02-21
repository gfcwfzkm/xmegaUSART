/*
 * uart.c
 * Version 1
 * Created: 14.12.2020
 *  Author: gfcwfzkm
 */ 

#include "uart.h"

#define SERIAL_UBBRVAL(Baud)    ((((F_CPU / 16) + (Baud / 2)) / (Baud)) - 1)
#define SERIAL_2X_UBBRVAL(Baud) ((((F_CPU / 8) + (Baud / 2)) / (Baud)) - 1)

/**
 * @brief Interrupt-Empfangsfunktion (Interrupt Service Routine)
 */
void uart_receive_interrupt(BUF_UART_t *temp)
{
	uint8_t tmphead,data,status,lastRxError = 0;
		
	/* Sicherstellen das der Pointer überhaupt wohin zeigt! */
	if (temp == 0)	return;
	
	/* Statusregister abarbeiten */
	status = (*temp->hw_usart).DATA;
	if (status & USART_BUFOVF_bm)	lastRxError |= UART_OVERRUN_ERROR;
	if (status & USART_FERR_bm)		lastRxError |= UART_FRAME_ERROR;
	if (status & USART_PERR_bm)		lastRxError |= UART_PARITY_ERROR;
	
	/* UART Datenregister einlesen */
	data = (*temp->hw_usart).DATA;	
	
	/* Neuer Bufferindex berechnen und überprüfen */
	tmphead = (temp->rxBufHead + 1) & (temp->rxBufLen - 1);
	
	if (tmphead == temp->rxBufTail)
	{
		/* Fehler! Der Empfangsbuffer ist voll! */
		lastRxError |= UART_BUFFER_OVERFLOW;
	}
	else
	{
		lastRxError |= UART_DATA_AVAILABLE;
		/* Neuer Bufferindex speichern */
		temp->rxBufHead = tmphead;
		/* Empfangenes Byte im Buffer speichern */
		temp->rxBuffer[tmphead] = data;
		/* Empfangenes Byte ggf. überprüfen */
		if ( (temp->lookForChar != 0) && (temp->lookForChar == data) )
		{
			temp->specialCharFound = 1;
		}
	}
	temp->lastError |= lastRxError;
	
	/* Empfangs / Rx Interrupt-Flag löscht sich beim Lesevorgang von selber */
}

/**
 * @brief Interrupt-Sendefunktion (Interrupt Service Routine)
 */
void uart_transmit_interrupt(BUF_UART_t *temp)
{
	uint8_t tmptail;
		
	/* Sicherstellen das der Pointer überhaupt wohin zeigt! */
	if (temp == 0)	return;
	
	if (temp->txBufHead != temp->txBufTail)
	{
		/* Neuer Bufferindex berechnen und speichern */
		tmptail = (temp->txBufTail + 1) & (temp->txBufLen - 1);
		temp->txBufTail = tmptail;
		/* Byte aus dem Buffer ins UART Senderegister legen */
		(*temp->hw_usart).DATA = temp->txBuffer[tmptail];
		
	}
	else
	{
		/* Tx Buffer leer, Sendeinterrupt deaktivieren */
		(*temp->hw_usart).CTRLA &=~ USART_DREINTLVL_LO_gc;
	}
}

void uart_init(BUF_UART_t *uartPtr, USART_t *_usart, int8_t _bscaleFactor, uint16_t bselValue, uint8_t useDoubleSpeed, 
		enum UART_SETTING _setting, uint8_t *recivBuf,  const uint8_t recivBufLen, uint8_t *transBuf, const uint8_t transBufLen)
{
	/* UARTInstanz Pointer sauber zurücksetzten / definieren */
	uartPtr->rxBuffer = recivBuf;
	uartPtr->rxBufLen = recivBufLen;
	uartPtr->rxBufHead = 0;
	uartPtr->rxBufTail = 0;
	uartPtr->txBuffer = transBuf;
	uartPtr->txBufLen = transBufLen;
	uartPtr->txBufTail = 0;
	uartPtr->txBufHead = 0;
	uartPtr->lastError = 0;
	uartPtr->lookForChar = 0;
	uartPtr->specialCharFound = 0;
	uartPtr->hw_usart = _usart;
		
	(*_usart).BAUDCTRLA = (uint8_t)bselValue;
	(*_usart).BAUDCTRLB = ((_bscaleFactor & 0x0F) << USART_BSCALE_gp)|((bselValue & 0xF00) >> 8);
	
	/* UART Modus setzten */
	(*_usart).CTRLC = _setting;
		
	/* UART Status löschen */
	(*_usart).STATUS = 0;
		
	/* Sender und Empfänger aktivieren */
	(*_usart).CTRLB |= (USART_RXEN_bm | USART_TXEN_bm | (useDoubleSpeed ? USART_CLK2X_bm : 0));
	(*_usart).CTRLA |= USART_RXCINTLVL_LO_gc;
}

void uart_putc(BUF_UART_t *uartPtr, uint8_t data)
{
	uint8_t tmphead, tmpBufTail;
	
	
	tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
		
	do 
	{
		tmpBufTail = uartPtr->txBufTail;		
		asm("nop");	/* Warten bis Platz im Buffer ist */		
	}while(tmphead == tmpBufTail);
	
	/* Zu sendendes Byte in den Buffer legen */
	uartPtr->txBuffer[tmphead] = data;
	uartPtr->txBufHead = tmphead;
	
	/* 'Data Register Empty' Interrupt aktivieren */
	(*uartPtr->hw_usart).CTRLA |= USART_DREINTLVL_LO_gc;
}

void uart_send(BUF_UART_t *uartPtr, uint8_t *dataBuffer, uint8_t bytesToSend)
{
	/* Anzahl Bytes vom Buffer senden */
	for (uint8_t i = 0; i < bytesToSend; i++)
	{
		uart_putc(uartPtr, dataBuffer[i]);
	}
}

void uart_print(BUF_UART_t *uartPtr, char *data)
{
	/* String bis Null-Terminator senden */
	while(*data)
	{
		uart_putc(uartPtr, *data++);
	}
}

void uart_print_f(BUF_UART_t *uartPtr, const __flash char *progmem_s )
{
	char character;
	/* String bis Null-Terminator vom FLASH-Speicher senden */
	while( (character = *progmem_s++) )
	{
		uart_putc(uartPtr, character);
	}

}

UART_ERROR uart_rxStatus(BUF_UART_t *uartPtr)
{
	uint8_t lastRxError = uartPtr->lastError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		lastRxError |= UART_NO_DATA;
	}
	
	/* Aktuellster Status zurückgeben, ohne ihn zu löschen */
	return (lastRxError);
}

uint16_t uart_peek(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen);
	
	/* FIFO Buffer -> ältestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	return ((lastRxError << 8) | data);
}

uint16_t uart_getc(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen - 1);
	
	/* FIFO Buffer -> ältestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	/* Bufferindex speichern */
	uartPtr->rxBufTail = tmptail;
	uartPtr->specialCharFound = 0;
	
	uartPtr->lastError = UART_DATA_AVAILABLE;
	return ((lastRxError << 8) | data);
}

UART_ERROR uart_isTxFull(BUF_UART_t *uartPtr)
{
	uint8_t tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
	
	if (tmphead == uartPtr->txBufTail)	return UART_TX_BUFFER_FULL;
	return UART_DATA_AVAILABLE;
}

uint8_t uart_charDetected(BUF_UART_t *uartPtr)
{
	uint8_t tmpHead,tmpTail,tmpLen,txtlen = 0;
	
	if (uartPtr->specialCharFound)
	{
		tmpHead = uartPtr->rxBufHead;
		tmpTail = uartPtr->rxBufTail;
		tmpLen = uartPtr->rxBufLen - 1;
		
		do
		{
			tmpTail = (tmpTail + 1) & tmpLen;
			
			txtlen++;
			
			if (tmpTail == tmpHead)
			{
				txtlen = 0;
				break;
			}
		} while (uartPtr->rxBuffer[tmpTail] != uartPtr->lookForChar);
		
		uartPtr->specialCharFound = 0;
	}
	
	return txtlen;
}

void uart_searchForCharacter(BUF_UART_t *uartPtr, char charToSearch)
{
	uartPtr->specialCharFound = 0;
	uartPtr->lookForChar = charToSearch;
}