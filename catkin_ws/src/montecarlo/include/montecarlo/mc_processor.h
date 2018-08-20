/**
 * @file mc_processor.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

namespace montecarlo {

  using Bag = std::tuple<std::string, int, int>;

  class MCProcessor
  {
  public:
    MCProcessor(std::string bagdir);
    ~MCProcessor() = default;

    void start();

  private:
    std::string bagdir_;      ///< Directory where simulation bags are located

    std::vector<Bag> bags_;   ///< vector of Bag tuples to be processed
    std::vector<int> Nts_;    ///< vector of 'num of targets' parameters
    int M_;                   ///< number of trials for each Nt

    /**
     * @brief      Find all simulation trial ROS bags and parse into useful tuple data structure
     *
     * @return     vector of Bag tuples
     */
    std::vector<Bag> find_bags();

    /**
     * @brief      Gets the mc statistics.
     *
     * @param[in]  bags  vector of bag tuples
     * @param      Nts   vector of Nt used to parameterize the overall MC simulation
     * @param      M     number of trials per Nt
     */
    void get_mc_stats(const std::vector<Bag>& bags, std::vector<int>& Nts, int& M);

    void for_each_Nt();

    void process_trial(std::string bagpath, int Nt, int m);
    
  };

}