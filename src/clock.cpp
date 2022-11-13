#include "clock.h"

unsigned int Clock::n_clocks = 0;
std::unordered_map<std::string, double> Clock::data;
std::unordered_map<std::string, unsigned int> Clock::n_data;

void Clock::stop(){
    // cout << "Done updateEdgeLists() " << duration<float>(high_resolution_clock::now() - clock0).count() << " seconds" << endl;
    std::chrono::time_point<std::chrono::high_resolution_clock> endTimePoint = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _start_timepoint).count();
    // Store the data
    data[name] += duration;
    n_data[name]++;

    // Print the data (every frame)
    // std::cout << "Clock " << name << "\t"<< duration << " us,\t" << duration * 0.001 << "ms" << std::endl;
}

void Clock::printClocks(){
    /* Prints the average of the dourations measured by each named clock
     * Note: it must ignore the current clock as it is only instanciated to print the results
     * of other clocks */

    // Iterate the map and print the averages
    for (auto [c_name, c_data] : data){
        if (c_name != name)
            std::cout << "Clock " << c_name << ":\t" << c_data/n_data[c_name] << " us\t in "<< n_data[c_name] << " samples." << std::endl;
    }
}
