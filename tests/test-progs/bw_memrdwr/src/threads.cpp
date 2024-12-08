#include <iostream> 
#include <thread> 
#include <mutex> 
#include <stdlib.h> 
#include <sys/time.h> 
#include <time.h> 
#include <sys/mman.h> 
#include <sys/types.h>
#include <string.h>
//#include "mops.h"

using namespace std;
int threadCnt = 0;
pthread_mutex_t mutex_sync;
pthread_cond_t cond_sync;

enum op
{
    op_invalid,
    op_rd,
    op_wr,
    op_mix,
    op_bcopy,
    op_fwr,
    op_triad,
    op_last
};

uint op;
unsigned LOOP;
unsigned long long SIZE;
unsigned long time_start = 0;
unsigned long * time_diff;
uint cpus = 0;
struct timeval proc_start;

unsigned int **back_buffer;
unsigned int **read_buffer_a;
unsigned int **read_buffer_b;

void array_add(int *a, int *b, int *c, int tid, int threads, int num_values)
{
    for (int i = tid; i < num_values; i += threads) {
        c[i] = a[i] + b[i];
    }
}

unsigned long usec_diff(struct timeval *a, struct timeval *b)
{
    unsigned long usec;

    usec = (b->tv_sec - a->tv_sec) *1000000;
    usec += b->tv_usec - a->tv_usec;
    return usec;
}

void print_test(unsigned int thread_id)
{
    cout << "byebye world["<<thread_id<<"]"<<endl;
}

void wait_time_start()
{
    if (time_start != 0) {
        struct timeval cur_time;
        unsigned long diff;
        do {
            gettimeofday(&cur_time, NULL);
            diff = usec_diff(&proc_start, &cur_time);
        } while (diff < time_start);
        unsigned long usec = cur_time.tv_sec * 1000000 + cur_time.tv_usec;
        cout <<" cur time:"<< usec <<";end diff: "<<diff<<endl;
    }
}

void mem_test(unsigned int* buffer, unsigned int thread_id)
{
    unsigned int size = SIZE;
    unsigned int sum;
    volatile unsigned int * p = buffer;
    volatile unsigned int * lastone = p+SIZE/sizeof(unsigned int) - 16;
    struct timeval start, end;
    unsigned long diff;

    wait_time_start();

    pthread_mutex_lock(&mutex_sync);
    threadCnt--;

    if (threadCnt <= 0) {
        pthread_mutex_unlock(&mutex_sync);
        pthread_cond_broadcast(&cond_sync);
        //m5_reset_stats(0, 0);
    } else {
        pthread_cond_wait(&cond_sync, &mutex_sync);
        pthread_mutex_unlock(&mutex_sync);
    }
    gettimeofday(&start, NULL);

    for(int k=0; k < LOOP; k++) {
            
        p = buffer;
        lastone = p+SIZE/sizeof(unsigned int)-496;
        
        while (p <= lastone) {
            sum +=
    #define DOIT(i) p[i]+
            DOIT (0) DOIT(16) DOIT (32) DOIT (48) DOIT (64) DOIT (80) DOIT (96) DOIT (112)
            DOIT (128) DOIT (144) DOIT (160) DOIT (176) DOIT (192) DOIT (208) DOIT (224)
            DOIT (256) DOIT (272) DOIT (288) DOIT (304) DOIT (320) DOIT (336) DOIT (352)
            DOIT (368) DOIT (384) DOIT (400) DOIT (416) DOIT (432) DOIT (448) DOIT (464)
            p[480];
            p += 496;
        }
    }
    #undef DOIT

    gettimeofday(&end, NULL);
    pthread_mutex_lock(&mutex_sync);
    threadCnt++;
    if (threadCnt == 1) {
        pthread_mutex_unlock(&mutex_sync);
        //m5_dump_reset_stats(0, 0);
    } else {
        pthread_mutex_unlock(&mutex_sync);
    }
    diff=usec_diff(&start, &end);
    time_diff[thread_id] = diff;

}

int main(int argc, char *argv[])
{
    unsigned num_values;
    unsigned int**buffer;

    if (argc == 6) {
        time_start = atoi(argv[5]);
        gettimeofday (&proc_start, NULL);
    }

    if (argc != 5 && argc != 6) {
        cerr << "usage: " << argv[0] << " [size: 1K/4M/1G..] [core: 1/4..] [Loop] [op: rd/wr/mix]" << endl;
        cerr << "example: " << argv[0] << " 4M 4 1 rd" << endl;
        return -1;
    }

    char *size_str = argv[1];
    char size_unit = size_str[strlen(size_str)-1];
    uint size = atoi(size_str);

    if (size_unit == 'K') {
        SIZE = size * 1024;
    } else if (size_unit == 'M') {
        SIZE = size * 1024 * 1024;
    } else if (size_unit == 'G') {
        SIZE = size * 1024 * 1024 * 1024;
    } else {
        SIZE = size;
    }

    cout << "SIZE: " << SIZE << endl;

    cpus = atoi(argv[2]);
    cout << "Thread: " << cpus << endl;
    LOOP = atoi(argv[3]);
    cout << "LOOP: " << LOOP << endl;

    char *op_str = argv[4];
    if (!strcmp(op_str, "rd")) {
        op = op_rd;
    } else if (!strcmp(op_str, "wr")) {
        op = op_wr;
    } else if (!strcmp(op_str, "mix")) {
        op = op_mix;
    } else if (!strcmp(op_str, "fwr")) {
        op = op_fwr;
    } else if (!strcmp(op_str, "bcopy")) {
        op = op_bcopy;
    } else if (!strcmp(op_str, "triad")) {
        op = op_triad;
    } else {
        return -1;
    }

    cout << "Op: " << op_str << endl;
    if (!cpus) {
        cpus = thread::hardware_concurrency();
    }
    cout << "Running on " << cpus << " cores." << endl;


    pthread_mutex_init(&mutex_sync, NULL);
    pthread_cond_init(&cond_sync, NULL);

    //SIZE = 1 * 1024 * 1024;

    cout << "SIZE: " << SIZE << endl;

    //cpus = 1;
    cout << "Thread: " << cpus << endl;
    //LOOP = 1;
    cout << "LOOP: " << LOOP << endl;

    //op = op_rd;
    cout << "OP: " << op << endl;

    //cout << "Op: " << op_str << endl;
    if (!cpus) {
        cpus = thread::hardware_concurrency();
    }
    cout << "Running on " << cpus << " cores." << endl;

    thread *threads = new thread[cpus];
    time_diff = new unsigned long[cpus];


    buffer = new unsigned int* [cpus];
    threadCnt = cpus;

    if (op == op_bcopy) {
        back_buffer = new unsigned int* [cpus];
    }
    if (op == op_triad) {
        read_buffer_a = new unsigned int* [cpus];
        read_buffer_b = new unsigned int* [cpus];
    }

    for (int i = 1; i < cpus ; i++) {
        buffer[i] = (unsigned int *)mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);

        if (op == op_bcopy) {
            back_buffer[i] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
        }
        
        if (op == op_triad) {
            read_buffer_a[i] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
            read_buffer_b[1] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
        }
        threads[i] = thread(mem_test, buffer[i], i);
    }

    cout << "start test" << endl;
    buffer[0] = (unsigned int *)mmap(0, SIZE, PROT_READ | PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);

    if (op == op_bcopy) {
        back_buffer[0] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
    }
    if (op == op_triad) {
        read_buffer_a[0] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
        read_buffer_b[0] = (unsigned int *)mmap(0, SIZE, PROT_READ|PROT_WRITE, MAP_ANONYMOUS | MAP_SHARED, -1, 0);
    }

    mem_test(buffer[0], 0);

    cout << "Waiting for other threads to complete" << endl;

    for(int i=1; i< cpus; i++){
        struct timeval time;
        gettimeofday(&time, NULL);
        cout <<"join thread " << i << " time "<< time.tv_sec << time.tv_usec<<endl;
        if (threads[i].joinable()) {
            try {
                threads[i].join();
            } catch (const std::exception& ex) {
                cout <<"exc!!!!!"<<&ex<<endl;
            }
        }
    }

    for(int i=0; i<cpus; i++) {
        double GBTran = 1e9;
        double bw = (double)SIZE*LOOP/time_diff[i] * 1000000 / GBTran;
        cout<<"time diff[i]: "<< time_diff[i]<<";"<<endl;
        cout<<"time["<<i<<"] = "<<bw<<"GBps"<<endl;
    }
    //delete[] threads;
    //delete [] buffer;
    cout << "Validating..." << flush;
    return 0;
}