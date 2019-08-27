/*
    Note the server doesn't need to know the clients port number
    but the client needs to the servers port number
*/

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>

using namespace std;

int main()
{
    int client, server;
    int portNum = 1501; // note the server and clients Ip are the same
    bool isExit = false;
    int bufsize = 1024;
    char buffer[bufsize];
//  char *ip = "127.0.0.1";

    struct sockaddr_in server_addr;

    //init socket

    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0)
    {
        cout<<"\n=> Error creating socket."<<endl;
        exit(1);
    }

    cout<<"\n=> Socket client has been created" <<endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port =  htons(portNum);

    //connecting socket server
    int res = connect(client, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if(res == 0)
    {
        cout << "=> Connecting to server..."<< endl;
    } else if(res == -1){
        cout<<"connect error"<<std::endl;
        return -1;
    } else{
        std::cout<<"res: "<<std::endl;
    }

    recv(client, buffer, bufsize, 0);
    cout<< "=> Connection confirmed" << endl;

    cout<< "\n=> Enter # to end the connection\n" << endl;

    do{
        cout<<"Client: "<<std::endl;
        do{
            cin >> buffer;
            send(client, buffer, bufsize, 0);
            if(*buffer == '#')
            {
                send(client, buffer, bufsize, 0);
                *buffer = '*';
                isExit = true;
            }
        } while(*buffer != 42);

        cout<<"Server: "<<std::endl;

        do{
            recv(client, buffer, bufsize, 0);
            cout <<"Server: "<< buffer << std::endl;
            if(*buffer == '#')
            {
                *buffer = '*';
                isExit = true;
            }
        }while(*buffer != 42);

        cout<<endl;
    }while(!isExit);

    cout<<"\nConnection terminated..."<< endl;
    cout<<"Goodbye..." << endl;

    close(client);

    return 0;
}