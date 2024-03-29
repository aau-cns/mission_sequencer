// Copyright (C) 2022 Alessandro Fornasier, Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <alessandro.fornasier@aau.at>
// and <martin.scheiber@aau.at>.

#ifndef MS_PARSER_WAYPOINT_HPP_
#define MS_PARSER_WAYPOINT_HPP_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "types/sequencer_waypoint.hpp"

namespace mission_sequencer
{
/// \brief Input data parser for the amaze waypoint following.
///
/// This class has a series of functions that allows to generate
/// data starting from a waypoint file for which convention is that
/// the first row contains the header defining what data is at each column.
/// Default convention --> x,y,z,yaw
///
/// \todo(rj): filename should correspond to class name.
class WaypointParser
{
public:
  ///
  /// \brief constructor
  ///
  WaypointParser();

  ///
  /// \brief constructor
  ///
  WaypointParser(const std::string& filename, const std::vector<std::string>& categories);

  ///
  /// \brief Clear actual data, read a new .csv file and convert to a matrix (vector of vectors)
  ///
  void readParseCsv();

  ///
  /// \brief Get Data red from data structure
  ///
  const std::vector<Waypoint>& getData() const;

private:
  ///
  /// \brief Filename of file containing waypoints
  ///
  std::string filename_;

  ///
  /// \brief Raw data from a .csv file converted to a matrix (vector of inputs)
  ///
  std::vector<Waypoint> data_;

  ///
  /// \brief vector of strings in header ordered based on defined convention -- x,y,z,yaw,holdtime --
  ///
  std::vector<std::string> categories_ = { "x", "y", "z", "yaw", "holdtime" };

  ///
  /// \brief Parse a single line of the .csv file
  ///
  /// overloaded function to parse a single line of a .csv
  /// file with a comma as delimeter.
  /// This function is overloaded to include either string values
  /// (usually the case for headers) or numerical values
  ///
  void parseLine(const std::string& line, std::vector<std::string>* const data);

  template <typename T>
  void parseLine(const std::string& line, std::vector<T>* const data)
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Temporary value
    T tmp;

    // Extract each cell
    while (ss >> tmp)
    {
      data->push_back(tmp);

      // skip commas
      if (ss.peek() == ',')
        ss.ignore();
    }
  }

  ///
  /// \brief Find association between input file and defined convention
  ///
  /// The defined convention of the Input structure is -- x,y,z,yaw,holdtime --
  /// This function find the indices of the columns of the input file based
  /// on its header in order to correctly associate input data with the
  /// Input structure allowing inpput files with shuffled columns or even
  /// more columns than the onse that are necessary
  ///
  void getIndices(const std::vector<std::string>& header, std::vector<int>* const indices);

  ///
  /// \brief Find the index of token within the given vector
  ///

  ///
  /// \brief getIndex Find the index of token within the given vector
  /// \param data Data vector to search for token
  /// \param token Token to search for in data
  /// \param index (return value) Index at which data was first found
  ///
  template <typename T>
  void getIndex(const std::vector<T>& data, const T& token, int* const index)
  {
    // Iterator
    auto it = find(data.begin(), data.end(), token);

    // Check if element was found
    if (it != data.end())
    {
      // Get the index
      *index = it - data.begin();
    }
    else
    {
      throw std::runtime_error("Required data missing. Exit programm.");
    }
  }
};

}  // namespace mission_sequencer

#endif  // MS_PARSER_WAYPOINT_HPP_
