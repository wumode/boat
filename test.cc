#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include "transform.h"
#include <thread>
#include <csignal>
#include <chrono>
#include <timer.h>
#include <ctimer.h>

using namespace std;
static void on_signal_term(int sig){
    cout << "on SIGTERM:" << std::this_thread::get_id() << endl;
    pthread_exit(NULL);
}
void threadPosixKill(void){
    signal(SIGTERM, on_signal_term);
    thread* t = new thread( [](){
        int counter = 0;
        while(true){
            ++counter;
            std::cout<<counter<<std::endl;
            this_thread::sleep_for( chrono::microseconds(5000) );
        }
    });
    pthread_t tid = t->native_handle();
    cout << "tid=" << tid << endl;
    // 确保子线程已经在运行。
    this_thread::sleep_for( chrono::seconds(1) );
    pthread_kill(tid, SIGTERM);
    t->join();
    delete t;
    cout << "thread destroyed." << endl;
}
double tests[][4] = {
        // wgsLat, wgsLng, gcjLat, gcjLng
        {37.52586, 122.07697, 37.53267730351087, 122.08859696767021}, // shanghai
        {22.543847, 113.912316, 22.540796131694766, 113.9171765808363}, // shenzhen
        {39.911954, 116.377817, 39.91334545536069, 116.38404722455657}, // beijing
};

class Test1{
public:
    Test1();
    ~Test1();

private:
    int* a;
};

Test1::Test1() {
    a = new int;
    *a = 8;
}

Test1::~Test1() {
    std::cout<<"free a: "<<*a<<std::endl;
    delete a;
}

void didHeartbeatThread(int arg){
    Test1 t1;
    std::cout<<"a "<<arg<<std::endl;
}


void didHeartbeatThread1(int arg){
    Test1 t2;
    std::cout<<"b "<<arg<<std::endl;
}

int main() {
    clock_t b;
    clock_t e;
    std::chrono::steady_clock::time_point st;
    std::chrono::steady_clock::time_point en;

    int t;
    int arg = 8;
    auto *pTimer = new CTimer("timer1");
    auto *pTimer2 = new CTimer("timer2");
    pTimer->AsyncLoop(100, didHeartbeatThread, arg);	//异步循环执行，间隔时间10毫秒
    pTimer2->AsyncLoop(5000, didHeartbeatThread1, arg);
    st = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(chrono::seconds(1));
    pTimer->Cancel();
    pTimer2->Cancel();
    en = std::chrono::steady_clock::now();
    std::chrono::duration<double> debug_time_used = std::chrono::duration_cast<std::chrono::duration<double>>(en-st);
    double debug_time_count = debug_time_used.count();
    printf("%f s\n", debug_time_count);

    //while(true); // Keep main thread active
    //threadPosixKill();
    return 0;
    size_t s = sizeof(tests) / sizeof(tests[0]);
    int i;

    for (i = 0; i < s; i++) {
        double wgsLat = tests[i][0], wgsLng = tests[i][1];
        double gcjLat, gcjLng;
        wgs2gcj(wgsLat, wgsLng, &gcjLat, &gcjLng);
        char got[1024], target[1024];
        sprintf(got, "%.06f,%.06f", gcjLat, gcjLng);
        sprintf(target, "%.06f,%.06f", tests[i][2], tests[i][3]);
        if (strcmp(got, target) != 0) {
            printf("wgs2gcj test %d: %s != %s\n", i, got, target);
        }
    }

    for (i = 0; i < s; i++) {
        double gcjLat = tests[i][2], gcjLng = tests[i][3];
        double wgsLat, wgsLng;
        gcj2wgs(gcjLat, gcjLng, &wgsLat, &wgsLng);
        double d = distance(wgsLat, wgsLng, tests[i][0], tests[i][1]);
        if (d > 5) {
            printf("gcj2wgs test %d: distance %f\n", i, d);
        }
    }

    for (i = 0; i < s; i++) {
        double gcjLat = tests[i][2], gcjLng = tests[i][3];
        double wgsLat, wgsLng;
        gcj2wgs_exact(gcjLat, gcjLng, &wgsLat, &wgsLng);
        double d = distance(wgsLat, wgsLng, tests[i][0], tests[i][1]);
        if (d > 0.5) {
            printf("gcj2wgs_exact test %d: distance %f\n", i, d);
        }
    }


    double lat, lng;
    int n;

    b = clock();
    for (i = 0; i < 10; i++) {
        wgs2gcj(tests[0][0], tests[0][1], &lat, &lng);
    }
    e = clock();
    n = (int)(CLOCKS_PER_SEC / (e - b)) * 10;
    b = clock();
    for (i = 0; i < n; i++) {
        wgs2gcj(tests[0][0], tests[0][1], &lat, &lng);
    }
    e = clock();
    t = (int)(((double)(e - b) / CLOCKS_PER_SEC) * 1e9 / n);
    printf("wgs2gcj\t%d\t%d ns/op\n", n, t);
    printf("lat: %0.6f-lng: %0.6f\n" , lat, lng);
    b = clock();
    for (i = 0; i < 10; i++) {
        gcj2wgs(tests[0][0], tests[0][1], &lat, &lng);
    }
    e = clock();
    n = (int)(CLOCKS_PER_SEC / (e - b)) * 10;
    b = clock();
    for (i = 0; i < n; i++) {
        gcj2wgs(tests[0][2], tests[0][3], &lat, &lng);
    }
    e = clock();
    t = (int)(((double)(e - b) / CLOCKS_PER_SEC) * 1e9 / n);
    printf("gcj2wgs\t%d\t%d ns/op\n", n, t);
    printf("lat: %0.6f-lng: %0.6f\n" , lat, lng);
    b = clock();
    for (i = 0; i < 10; i++) {
        gcj2wgs_exact(tests[0][0], tests[0][1], &lat, &lng);
    }
    e = clock();
    n = (int)(CLOCKS_PER_SEC / (e - b)) * 10;
    b = clock();
    for (i = 0; i < n; i++) {
        gcj2wgs_exact(tests[0][2], tests[0][3], &lat, &lng);
    }
    e = clock();
    t = (int)(((double)(e - b) / CLOCKS_PER_SEC) * 1e9 / n);
    printf("gcj2wgs_exact\t%d\t%d ns/op\n", n, t);
    printf("lat: %0.6f-lng: %0.6f\n" , lat, lng);
    b = clock();
    for (i = 0; i < 10; i++) {
        distance(tests[0][0], tests[0][1], tests[0][2], tests[0][3]);
    }
    e = clock();
    n = (int)(CLOCKS_PER_SEC / (e - b)) * 10;
    b = clock();
    for (i = 0; i < n; i++) {
        distance(tests[0][0], tests[0][1], tests[0][2], tests[0][3]);
    }
    e = clock();

}