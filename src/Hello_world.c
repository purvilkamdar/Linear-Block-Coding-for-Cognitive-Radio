#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


int m=8;
int n=12;
int G[8][12]={{1,0,0,0,0,0,0,0,1,1,0,0},
                            {0,1,0,0,0,0,0,0,0,1,1,0},
                            {0,0,1,0,0,0,0,0,0,0,1,1},
                            {0,0,0,1,0,0,0,0,1,0,0,1},
                            {0,0,0,0,1,0,0,0,1,0,1,0},
                            {0,0,0,0,0,1,0,0,0,1,0,1},
                            {0,0,0,0,0,0,1,0,1,1,1,0},
                            {0,0,0,0,0,0,0,1,0,1,1,1}};
int H[12][4];
int confidence_level;
int confidence_match;
int counter;
int destination_start_bit;
int payload_start_bit;
int endbit;
int order_level;
int scramble_choice;
int buffer[1024];
int recieve_done=0;
int acknowledgment_buffer[2048];
int acknowledgment_counter=0;
int sendData[1024]={  0,1,0,1,0,0,0,0,
					  0,1,0,1,0,0,0,1,
					  0,1,0,1,0,0,1,0,
					  0,1,0,1,0,0,1,1,
					  0,1,0,1,0,1,0,0,
					  0,1,0,1,0,1,0,1,
					  0,1,0,1,0,1,1,0,
					  0,1,0,1,0,1,1,1,
					  0,1,0,1,1,0,0,0,
					  0,1,0,1,1,0,0,1,
					  0,1,0,1,1,0,1,0,
					  0,1,0,1,1,0,1,1,
					  0,1,0,1,1,1,0,0,
					  0,1,0,1,1,1,0,1,
					  0,1,0,1,1,1,1,0,
					  0,1,0,1,1,1,1,1,
					  0,0,0,1,0,1,1,0,0,0,0,0,
					  0,0,0,0,1,0,1,1,0,0,0,0,
					  0,1,0,1,0,0,0,0,0,0,0,0,
					  0,1,0,1,0,1,0,1,0,0,0,0,
					  0,1,0,1,0,0,1,0,0,0,0,0,
					  0,1,0,1,0,1,1,0,0,0,0,0,
					  0,1,0,0,1,0,0,1,0,0,0,0,
					  0,1,0,0,1,1,0,0,0,0,0,0,
					  0,0,1,0,0,0,0,0,0,0,0,0,
					  0,1,0,0,1,0,1,1,0,0,0,0,
					  0,1,0,0,0,0,0,1,0,0,0,0,
					  0,1,0,0,1,1,0,1,0,0,0,0,
					  0,1,0,0,0,1,0,0,0,0,0,0,
					  0,1,0,0,0,0,0,1,0,0,0,0,
					  0,1,0,1,0,0,1,0,0,0,0,0,
					  0,0,1,0,0,0,0,0,0,0,0,0,
					  0,0,1,1,0,0,1,1,0,0,0,0,
					  0,0,1,1,0,1,1,1,0,0,0,0,
					  0,0,1,1,0,0,1,0,0,0,0,0,
					  0,0,1,1,0,0,1,0,0,0,0,0,
                      0,0,1,0,0,0,1,1,
					  0,0,1,0,0,0,1,1};

uint8_t arr[32]={0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x5f,0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf};

void GPIOinitIn(uint8_t portNum, uint32_t pinNum)
{
	if (portNum == 0)
	{
		LPC_GPIO0->FIODIR &= ~(1 << pinNum);
	}
	else if (portNum == 1)
	{
		LPC_GPIO1->FIODIR &= ~(1 << pinNum);
	}
	else if (portNum == 2)
	{
		LPC_GPIO2->FIODIR &= ~(1 << pinNum);
	}
	else
	{
		puts("Not a valid port!\n");
	}
}
void GPIOinitOut(uint8_t portNum, uint32_t pinNum)
{
	if (portNum == 0)
	{
		LPC_GPIO0->FIODIR |= (1 << pinNum);
	}
	else if (portNum == 1)
	{
		LPC_GPIO1->FIODIR |= (1 << pinNum);
	}
	else if (portNum == 2)
	{
		LPC_GPIO2->FIODIR |= (1 << pinNum);
	}
	else
	{
		puts("Not a valid port!\n");
	}
}
void setGPIO(uint8_t portNum, uint32_t pinNum)
{
	if (portNum == 0)
	{
		LPC_GPIO0->FIOSET = (1 << pinNum);
		//printf("Pin 0.%d has been set.\n",pinNum);
	}
	//Can be used to set pins on other ports for future modification
	else
	{
		puts("Only port 0 is used, try again!\n");
	}
}
void clearGPIO(uint8_t portNum, uint32_t pinNum)
{
	if (portNum == 0)
	{
		LPC_GPIO0->FIOCLR = (1 << pinNum);
		//printf("Pin 0.%d has been cleared.\n", pinNum);
	}
	//Can be used to clear pins on other ports for future modification
	else
	{
		puts("Only port 0 is used, try again!\n");
	}
}

void delay_ms(uint32_t delayInMs) {
	LPC_TIM0->TCR = 0x02;
	LPC_TIM0->PR = 0x00;
	LPC_TIM0->MR0 = delayInMs * 0.5 * (9000000 / 1000 - 1);
	LPC_TIM0->IR = 0xff;
	LPC_TIM0->MCR = 0x04;
	LPC_TIM0->TCR = 0x01;

	while (LPC_TIM0->TCR & 0x01)
		;
	return;
}



int lisa_match_for_ack(int confidence)
{
	//printf("Entered Lisa match \n");
	if(counter>31)
	{
		printf("Acknowledgment not recieved");
		return 0;
	}
	int startbit=0;
	uint8_t c;
	int i,j;
	int matched=0;
	int bit=0;
    for(i=0;i<2048;i++)
    {
		if(confidence_match==confidence)
		{
			printf("Acknowledgment recieved");
            return 0;
			//exit(0);
		}
 	   for(j=i;j<i+8;j++)
 	   {	   c=arr[counter];
 		   c=c << (j-i);
 		   c=c >> 7;
 		   if((c^acknowledgment_buffer[j])!=0)
 			{
				matched=0;
 				break;
 			}
			if(j==(i+7))
			{
				matched=1;
			}
 		}
		if(matched==1)
		{
			i=i+7;
			confidence_match++;
			counter++;
		}
    }
	counter++;
	confidence_match=0;
	printf("Acknowledgment not recieved");
	return 0;
}


void sendAck()
{
	GPIOinitOut(0,2);
	int r=0;
	int b=0;
	for(b=0;b<16;b++){
	for(r=0;r<128;r++){
		delay_ms(100);
		if(sendData[r]==1)
			{
				setGPIO(0,2);
			}
			else
			{
				clearGPIO(0,2);
			}
		}
	}
}

void receiveAck()
{

	int one_count=0;
	int zero_count=0;
	int r=0;
	int b=0;
	GPIOinitIn(0,3);
	clearGPIO(0,3);
	for (r = 0; r < 2048; r++) {

		delay_ms(100);
		if (LPC_GPIO0->FIOPIN0 & (1 << 3)) {
			acknowledgment_buffer[r] = 1;
			}
		else {
			acknowledgment_buffer[r] = 0;
			}

	}
	lisa_match_for_ack(2);
	/*for(r=0;r<1024;r++)
	{
		printf("%d,",acknowledgment_buffer[r]);
	}*/


	return;
}







void print_payload(int start, int stop);

int end_value()
{

			int matched=0;
			int i,j,k;
			uint8_t c;
			uint8_t ending_value=0x23;
		    for(i=0;i<1024;i++)
		    {
		 	   for(j=i;j<i+8;j++)
		 	   {
					c=ending_value;
		 		   c=c << (j-i);
		 		   c=c >> 7;
		 		   if((c^buffer[j])!=0)
		 			{
		 				break;
		 			}
					if(j==(i+7))
					{
						matched++;
						if(matched==1)
						{
							for(k=i+8;k<i+16;k++)
		 	   				{
								c=ending_value;
		 		   				c=c << (k-(i+8));
		 		   				c=c >> 7;
		 		   				if((c^buffer[k])!=0)
		 						{
									matched=0;
		 							break;
								}
								if(k==(i+15))
								{
									matched++;
		 						}
							}
						}
		 			}
				if(matched==2)
				{
					printf("End pattern matched \n");
					endbit=i;
					printf("endbit is %d",endbit);
					return endbit;
				}
		    }

			}
		printf("Error: end pattern not found");
		exit(0);
}

void scramble(const int start, const int stop)
{
	counter=0;
	int st=start;
	int sp=stop;
	//printf("Start: %d, Stop: %d \n",start,stop);
	int i,j,k,temp,counter=0,l=0;
	int d1[order_level/2];
	int d2[order_level];
	int scrambled[(stop-start+1)];
	for(i=0;i<(order_level/2);i++)
	{
		d1[i]=0;
	}
	for(i=0;i<(order_level);i++)
	{
		d2[i]=0;
	}
	i=0;
	j=0;
	k=0;
	//printf("The data value is:");
	for(i=st;i<sp+1;i=i+12)
	{
		//printf("i:%d",i);
		for(l=0;l<(order_level/2);l++)
		{
			d1[l]=0;
		}
		for(l=0;l<(order_level);l++)
		{
			d2[l]=0;
		}

		//d1[(order_level/2)-1]=0;
		//d2[order_level-1]=0;
		//printf("The sendData value is:");
		for(k=i;k<i+8;k++)
		{	scrambled[counter]=sendData[k]^d1[(order_level/2)-1]^d2[order_level-1];
			//printf("i value: %d, %d",i,sendData[k]);
			for(j=(order_level/2)-1;j>=1;j--)
			{
				temp=d1[j-1];
				d1[j]=temp;
			}
			d1[0]=scrambled[counter];
			for(j=(order_level)-1;j>=1;j--)
			{
				temp=d2[j-1];
				d2[j]=temp;
			}
			d2[0]=scrambled[counter];
			sendData[k]=scrambled[counter];
			//printf("%d,",sendData[k]);
			counter++;
		}//printf("\n");

	}
	//printf("\nThe scrambled value is:");
	/*j=0;
	//printf("Start value is %d and Stop value is %d \n",st,sp);
	for(i=st;i<sp+1;i++)
	{
		//sendData[i]=scrambled[j];
		printf("%d,",i);
		j++;
		if(j%12==0)
			printf("\n");
	}*/
	//printf("\n");
}


void create_syndrome_word(int start, int stop)
{
	int st=start;
	int sp=stop;
    int i=0;
    int j=0;
    int k=0;
    int temp=0;
    int counter=0;
    int temp_array[400];
	/*printf("Senddata=");
	for(i=start;i<288;i++)
	{
		printf("%d,",sendData[i]);
		counter++;
		if(counter%8==0)
			printf("\n");
	}
	printf("\n");*/
	counter=0;
    for(k=st;k<sp+1;k=k+12)
    {
        for(i=0;i<n;i++)
        {
            temp=0;
            for(j=0;j<m;j++)
            {
				//printf("%d,",k+j);
				//printf("%d,",sendData[k+j]);
                temp=temp+sendData[k+j]*G[j][i];
            }
            temp_array[counter]=(temp%2);
			//printf("\n");
			//printf("%d,",temp_array[counter]);
			counter++;
        }
		//printf("\n");
    }
	//printf("counter=%d",counter);
	//printf("\n");
	counter=0;
    for(i=st;i<sp+1;i++)
    {
        sendData[i]=temp_array[counter];
		counter++;
		//printf("%d,",sendData[i]);
		//if(counter%12==0)
			//printf("\n");
    }
	/*counter=0;
	for(i=start;i<stop+40;i++)
    {
        printf("%d,",sendData[i]);
		counter++;
		if(counter%12==0)
			printf("\n");
    }*/
}
void transpose()
{
    int i=0;
    int j=0;
    for(i=0;i<n;i++)
    {
        for(j=m;j<n;j++)
        {
            H[i][j-m]=G[i][j];
        }
    }
    for(i=m;i<n;i++)
    {
        for(j=0;j<(n-m);j++)
        {
            if((i%m)==j)
                H[i][j]=1;
            else
                H[i][j]=0;
        }
    }

}

void check_error(int start, int stop)
{
	//printf("Entered check_error");
	int st=start;
	int sp=stop;
	int i=0;
    int j=0;
	int k=0;
    int temp=0;
    for(k=st;k<sp+1;k=k+12)
	{
		for(i=0;i<(n-m);i++)
    	{
        	temp=0;
        	for(j=0;j<n;j++)
        	{
				//printf("%d,",sendData[k+j]);
            	temp=temp+buffer[k+j]*H[j][i];
       	 	}
			//printf("\n");
			//printf("temp=%d",temp);
			if(temp%2==1)
				{
					printf("There is an error");
					exit(0);
				}
       	 	//decoded[i]=(temp%2);
    	}
	}
    printf("No error");

}


void descramble(const int start, const int stop)
{
	//printf("Entered descramble\n");
	int st=start;
	int sp=stop;
	int i,j,k,temp,counter=0,l;
	int d1[order_level/2];
	int d2[order_level];
	int descrambled[(sp-st+1)];

	for(i=0;i<(order_level/2);i++)
		d1[i]=0;
	for(i=0;i<(order_level);i++)
		d2[i]=0;

	j=0;
	k=0;
	//printf("\ndescramble=");
	/*counter=0;
	for(i=st;i<sp+1;i++)
	{
		printf("%d,",i);
		counter++;
		if(counter%12==0)
			printf("\n");
	}*/
	counter=0;
	for(i=st;i<sp+1;i=i+12)
	{
		for(l=0;l<(order_level/2);l++)
				d1[l]=0;
			for(l=0;l<(order_level);l++)
				d2[l]=0;

		//d1[(order_level/2)-1]=0;
		//d2[order_level-1]=0;
		//printf("\nThe sendData value is:");
		for(k=i;k<i+8;k++)
		{	descrambled[counter]=buffer[k]^d1[(order_level/2)-1]^d2[order_level-1];

			//printf("%d",sendData[k]);
			for(j=(order_level/2)-1;j>=1;j--)
			{
				temp=d1[j-1];
				d1[j]=temp;
			}
			d1[0]=buffer[k];
			for(j=(order_level)-1;j>=1;j--)
			{
				temp=d2[j-1];
				d2[j]=temp;
			}
			d2[0]=buffer[k];
			//printf("%d,",sendData[k]);
			buffer[k]=descrambled[counter];
			counter++;
		}//printf("\n");
	}

	/*j=0;
	printf("\nThe descrambled value is:");
	for(i=st;i<sp+1;i++)
	{
		//sendData[i]=descrambled[j];
		printf("%d",sendData[i]);
		j++;
		if(j%8==0)
			printf("\n");
	}*/
}


void descramble1(const int start, const int stop)
{
	int st=start;
	int sp=stop;
	int i,j,k,temp,counter=0,l;
	int d1[order_level/2];
	int d2[order_level];
	int descrambled[(sp-st+1)];

	for(i=0;i<(order_level/2);i++)
		d1[i]=0;
	for(i=0;i<(order_level);i++)
		d2[i]=0;

	j=0;
	k=0;
	//printf("\ndescramble=");
	/*counter=0;
	for(i=st;i<sp+1;i++)
	{
		printf("%d,",i);
		counter++;
		if(counter%12==0)
			printf("\n");
	}*/
	counter=0;
	for(i=st;i<sp+1;i=i+12)
	{
		for(l=0;l<(order_level/2);l++)
				d1[l]=0;
			for(l=0;l<(order_level);l++)
				d2[l]=0;

		//d1[(order_level/2)-1]=0;
		//d2[order_level-1]=0;
		//printf("\nThe sendData value is:");
		for(k=i;k<i+8;k++)
		{	descrambled[counter]=sendData[k]^d1[(order_level/2)-1]^d2[order_level-1];

			//printf("%d",sendData[k]);
			for(j=(order_level/2)-1;j>=1;j--)
			{
				temp=d1[j-1];
				d1[j]=temp;
			}
			d1[0]=sendData[k];
			for(j=(order_level)-1;j>=1;j--)
			{
				temp=d2[j-1];
				d2[j]=temp;
			}
			d2[0]=sendData[k];
			//printf("%d,",sendData[k]);
			sendData[k]=descrambled[counter];
			counter++;
		}//printf("\n");
	}

	/*j=0;
	printf("\nThe descrambled value is:");
	for(i=st;i<sp+1;i++)
	{
		//sendData[i]=descrambled[j];
		printf("%d",sendData[i]);
		j++;
		if(j%8==0)
			printf("\n");
	}*/
}




void check_destination_id()
{
	uint8_t dest_id=0x16;
	uint8_t c;
	int i;
	int matched=0;
	c=dest_id;
	//printf("sendData value for destination id:");
	for(i=destination_start_bit;i<destination_start_bit+8;i++)
	{
		c=dest_id;
		c=c << (i-destination_start_bit);
		c=c >> 7;
		//printf("%d");
		if((c^buffer[i])!=0)
		{
			matched=0;
			break;
		 }
		if(i==(destination_start_bit+7))
		{
			matched=1;
			printf("\n Destination id of the incoming packet matched with the source id.\n");
		}
	}
	if(matched==0)
	{
		printf("\n");
		printf("Destination id didnot match");
        exit(0);
	}
}

int lisa_match(int endbit)
{
	//printf("Entered Lisa match \n");
	if(counter>31)
		return 0;
	int startbit=0;
	uint8_t c;
	int i,j;
	int matched=0;
	int bit=0;
    for(i=0;i<endbit;i++)
    {
		if(confidence_match==confidence_level)
		{
			bit=i-8;
			destination_start_bit=bit+140-((counter-1)*8);
			payload_start_bit=bit+152-((counter-1)*8);
			printf("Sync matched with the last pattern being %x\n", arr[counter-1]);
            printf("Destination start bit is %d\n",destination_start_bit);
			printf("payload start value is=%d\n",payload_start_bit);
			transpose();
            check_error(destination_start_bit, endbit-1);
			descramble(destination_start_bit, endbit-1);
			check_destination_id();
			print_payload(payload_start_bit, endbit-1);
			recieve_done=1;
			confidence_match=0;
			return 0;
			//exit(0);
		}
 	   for(j=i;j<i+8;j++)
 	   {	   c=arr[counter];
 		   c=c << (j-i);
 		   c=c >> 7;
 		   if((c^buffer[j])!=0)
 			{
				matched=0;
 				break;
 			}
			if(j==(i+7))
			{
				matched=1;
			}
 		}
		if(matched==1)
		{
			i=i+7;
			confidence_match++;
			counter++;
		}
    }
	counter++;
	confidence_match=0;
	return 0;
}

void print_payload(const int start, const int stop)
{
	//printf("\nPayload data is:");
	int st=start;
	int sp=stop;
	int i=0;
	int j=0;
	int counter=0;
	int charc=0;
	for (i=st;i<sp+1;i=i+12)
	{
		charc=0;
		counter=0;
		//printf("data=");
		/*for(j=i;j<i+8;j++)
		{
			printf("%d,",sendData[j]);
		}*/
		for (j=i+7;j>=i;j--)
		{
			//printf("%d,",j);
			charc=charc+(buffer[j]*pow(2,(counter)));
			counter++;
		}
		//printf("\n");
		printf("%c",charc);
	}
	printf("\n");
}

void receive()
{
    int i,j;
	end_value();
	//printf("Entered recieve \n");

	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match &&!recieve_done)
	{
			 lisa_match(endbit);
			 if(recieve_done)
			 {
				 recieve_done=0;
				 return;
			 }
	}
	if(confidence_level!=confidence_match)
	{
		printf("Error: Required confidence_level was not matched");
        exit(0);
		//return;
	}
}

int main(void)
{
    // Force the counter to be placed into memory
    volatile static int i = 0 ;
	int j=0;
    int status=0;
	int counter=0;
    //Set pin 0.2 as output

	//Set pin 0.3 as input

    int p;
    while(1)
    {
		printf("Enter the confidence level: \n");
		scanf("%d",&confidence_level);
		printf("Enter the order for scrambling: \n");
		scanf("%d",&order_level);
		printf("Enter 1 for scrambling and 2 for descrambling: \n");
		scanf("%d", &scramble_choice);
		if(scramble_choice!= 1 && scramble_choice!= 2)
		{
			printf("Invalid input for choice");
			return 0;
		}
		if(scramble_choice==1)
		{

			scramble(128,367);
            create_syndrome_word(128,367);
		}
		if(scramble_choice==2)
		{
			GPIOinitOut(0,2);
			clearGPIO(0,2);


		}
    	printf("Enter 1 for send and 2 for receive.\n");
        scanf("%d", &i);

		if (i == 1)
		{
			GPIOinitOut(0,2);
			//Activate pin 0.2

			//scramble(128,287);
			//setGPIO(0,2);
			for(p=0;p<400;p++){
				delay_ms(10);
				if(sendData[p]==1)
				{
					setGPIO(0,2);
				}
				else
				{
					clearGPIO(0,2);
				}

				//descramble1(128,287);
				//receiveAck();
				//if(LPC_GPIO0->FIOPIN0 & (1<<3))
				//{
					//buffer[p]=1;
				//}
				//else
				//{
					//buffer[p]=0;
				//}

			}
			delay_ms(10);
			receiveAck();
			descramble1(128,367);

		}
		else if (i == 2)
		{
			if(confidence_level>0 && confidence_level<31 && order_level%2==1 && order_level>0)
			{

			}
			else
			{
				printf("Invalid input");
				exit(0);
			}
			GPIOinitIn(0,3);
					for (p = 0; p < 1024; p++) {

					delay_ms(10);
					if (LPC_GPIO0->FIOPIN0 & (1 << 3)) {
						buffer[p] = 1;
					} else {
						buffer[p] = 0;
					}

				}
						/*printf("\nReceived Data=\n");
						for(p=0;p<1024;p++)
						{
							printf("%d",buffer[p]);
						}

						printf("\n");
*/
			receive();
			delay_ms(10);
			//sendAck();
            sendAck();
		}
    }
    return 0;
}
