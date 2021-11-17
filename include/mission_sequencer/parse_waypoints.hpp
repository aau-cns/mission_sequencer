/*
 * Alessandro Fornasier (CNS)
 * alessandro.fornasier@aau.at
 */

#ifndef PARSEWAYPOINT_HPP
#define PARSEWAYPOINT_HPP

#include <stdlib.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief Input data parser for the amaze waypoint following.
 *
 * This class has a series of functions that allows to generate
 * data starting from a waypoint file for which convention is that
 * the first row contains the header defining what data is at each column.
 * Default convention --> x,y,z,yaw
 *
 */

class ParseWaypoint
{
public:
  /**
   * @brief Input struct to parse a single line of a .csv file
   *
   * x
   * y
   * z
   * yaw
   */
  struct Waypoint
  {
    double x;
    double y;
    double z;
    double yaw;
    double holdtime;
  };

  /**
   * @brief constructor
   */
  ParseWaypoint();

  /**
   * @brief constructor
   */
  ParseWaypoint(std::string& filename, std::vector<std::string>& categories);

  /**
   * @brief Clear actual data, read a new .csv file and convert to a matrix (vector of vectors)
   */
  void readParseCsv();

  /**
   * @brief Get Data red from data structure
   */
  const std::vector<Waypoint>& getData() const;

private:
  /**
   * @brief Filename of file containing waypoints
   */
  std::string filename_;

  /**
   * @brief Raw data from a .csv file converted to a matrix (vector of inputs)
   */
  std::vector<Waypoint> data_;

  /**
   * @brief vector of strings in header ordered based on defined convention -- x,y,z,yaw,holdtime --
   */
  std::vector<std::string> categories_ = { "x", "y", "z", "yaw", "holdtime" };

  /**
   * @brief Parse a single line of the .csv file
   *
   * overloaded function to parse a single line of a .csv
   * file with a comma as delimeter.
   * This function is overloaded to include either string values
   * (usually the case for headers) or numerical values
   */
  void parseLine(std::string& line, std::vector<std::string>& data);

  template <typename T>
  void parseLine(std::string& line, std::vector<T>& data)
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Temporary value
    T tmp;

    // Extract each cell
    while (ss >> tmp)
    {
      data.push_back(tmp);

      // skip commas
      if (ss.peek() == ',')
        ss.ignore();
    }
  }

  /**
   * @brief Find association between input file and defined convention
   *
   * The defined convention of the Input structure is -- x,y,z,yaw,holdtime --
   * This function find the indices of the columns of the input file based
   * on its header in order to correctly associate input data with the
   * Input structure allowing inpput files with shuffled columns or even
   * more columns than the onse that are necessary
   */
  void getIndices(const std::vector<std::string>& header, std::vector<int>& indices);

  /**
   * @brief Find the index of token within the given vector
   */
  template <typename T>
  void getIndex(const std::vector<T>& data, const T& token, int& index)
  {
    // Iterator
    auto it = find(data.begin(), data.end(), token);

    // Check if element was found
    if (it != data.end())
    {
      // Get the index
      index = it - data.begin();
    }
    else
    {
      throw std::runtime_error("Required data missing. Exit programm.");
    }
  }
};

#endif  // PARSEWAYPOINT_HPP
