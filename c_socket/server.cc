#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

int main()
{
    int client, server;
    int portNum = 1501;
    bool isExit = false;
    int bufsize = 1024;
    char buffer[bufsize];

    struct sockaddr_in server_addr;
    socklen_t size;

    //init socket.

    client = socket(AF_INET, SOCK_STREAM, 0); //AF_INET指明使用TCP/IP协议簇
    //SOCK_STREAM指明使用流式嵌套字
    //使用default 协议类型

    if (client < 0) //返回值为-1，连接失败 
    {
        cout<<"=> Error establishing connection."<<endl;
        exit(1);
    }

    cout<<"\n=> Socket server has been created."<<endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY); //INADDR_ANY能够收到任意一块网卡的连接 
    server_addr.sin_port =  htons(portNum);

    //binding socket

    if(bind(client, (struct sockaddr*)&server_addr, sizeof(server_addr))<0)
    {
        cout<<"=> Error binding socket..."<<endl;
        exit(1);
    }

    size = sizeof(server_addr);
    cout<<"=> Looking for client"<<endl;

    //listening socket

    listen(client,1);

    //accept client

    server = accept(client, (struct sockaddr*)&server_addr,&size);

    if(server < 0)
    {
        cout<<"=> Error on accepting..."<<endl;
        exit(1);
    }

    while(server > 0)
    {
        strcpy(buffer,"Server connected...\n");
        send(server, buffer, bufsize, 0);
        cout<<"=> Connected with client..."<<endl;
        cout<<"\n=> Enter # to end the connection\n"<<endl;
        cout<<"\n=> Enter # to end the connection\n"<<endl;
        cout<<"Client: "<<std::endl;
        do{
            recv(server, buffer, bufsize, 0);
            cout << "Client: "<<buffer << endl;
            if(*buffer == '#')
            {
                *buffer = '*';
                isExit = true;
            }
        }while(*buffer != '*');

        do{
            cout<<"\nServer: "<<std::endl;
            do{
                cin >> buffer;
                send(server, buffer, bufsize, 0);
                if(*buffer == '#')
                {
                    send(server, buffer, bufsize, 0);
                    *buffer = '*';
                    isExit = true;
                }
            }while (*buffer != '*');

            cout<<"Client: "<<std::endl;
            do{
                recv(server, buffer, bufsize, 0);
                std::cout<<"Client: " << buffer <<std::endl;
                if(*buffer == '#')
                {
                    *buffer == '*';
                    isExit = true;
                }
            }while(*buffer != '*');
        }while(!isExit);

        cout<<"\n\n=> Connection terminated..."<< endl;
        cout<<"=> Goodbye..." << endl;
        isExit = false;
        exit(1);
    }
    close(client);
    return 0;
}