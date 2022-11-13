#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
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
        }
        ~Clock(){
            n_clocks--;
            stop();
        }
        void stop();
        void printClocks();

        std::string name;
    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> _start_timepoint;
};

#endif // CLOCK_H
