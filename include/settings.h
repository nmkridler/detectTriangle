#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <cstdlib>
#include <cv.h>

typedef std::map<std::string,std::string > fileMap;

class Settings
{
   public:
      // Constructor
      Settings(std::string const & fileName);

      // Destructor
      ~Settings(){}

      // Settings reader
      bool readSettings();

      // Settings are public
      cv::Point3d                dimensions;     // x: base, y: height z: hypotenuse
      cv::Point3d                color;          // x: red, y: green, z: blue
      double                     threshold;      // detection threshold
      int                        misses;         // number of allowable misses

   private:

      std::string                m_settingsFile;   // Text file containing settings


};

typedef boost::shared_ptr<Settings> SettingsPtr;
#endif
