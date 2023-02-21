#ifndef CLOCK_H
#define CLOCK_H

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

        bool print_each_frame = false;

        Clock(std::string name);
        ~Clock();

        void printClocks();

        std::string name;

    private:
        // Initial and final times
        timespec _itime;
        timespec _ftime;
};

#endif // CLOCK_H
