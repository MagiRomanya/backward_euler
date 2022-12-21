#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

class Clock{
    public:
        static unsigned int n_clocks;
        static std::unordered_map<std::string, double> data;
        static std::unordered_map<std::string, unsigned int> n_data;
        Clock(std::string name){
            n_clocks++;
            this->name = name;
            if (!data.contains(name)){
                data[name] = 0;
                n_data[name] = 0;
            }
            _start_timepoint = std::chrono::high_resolution_clock::now();
            // https://stackoverflow.com/questions/275004/timer-function-to-provide-time-in-nano-seconds-using-c/275231#275231
            clock_gettime(CLOCK_REALTIME, &_itime);
        }
        ~Clock(){
            n_clocks--;
            clock_gettime(CLOCK_REALTIME, &_ftime);
            // How to use gettime: https://www.educba.com/clock_gettime/
            auto diff = ( _ftime.tv_sec - _itime.tv_sec ) * (double) 1000000000L
                + (double)( _ftime.tv_nsec - _itime.tv_nsec ); // in nanoseconds
            //std::cout << diff/1000000000 << std::endl; // output in seconds
            stop();
        }
        void stop();
        void printClocks();

        std::string name;
    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> _start_timepoint;
        // Initial and final times
        timespec _itime;
        timespec _ftime;
};

#endif // CLOCK_H
