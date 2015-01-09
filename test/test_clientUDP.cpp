#include <nymeria_ardrone/UDPWrapper.h>
#include <stdlib.h>

int main()
{
	char message[10];
	int nb_char;
	UDPClient client("127.0.0.1", 7777);
	
	printf("test2\n");
	while(client.send("go", 2) <=0);
	while (1)
	{
		nb_char = client.recv(message, 10);
		if(message[0] >120)
		{
			message[0] = 128;
		}		
		printf("recu : %d-\n", message[0]);
	}
}
