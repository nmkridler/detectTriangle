#include <settings.h>

Settings::Settings( std::string const & fileName):
m_settingsFile(fileName)
{
}

//////////////////////////////////////////////////////////
//
// readSettings
//
// This function opens up the settings file and reads the
// list of input images to process.
// 
// The file should be formatted as follows
// dims    base, height
// color   red,green,blue
// numDet  number of detections
//
//////////////////////////////////////////////////////////
bool Settings::readSettings()
{
   // Create a map<string,string> for the files
   fileMap fMap;

   // Create an input stream
   std::ifstream inFile(m_settingsFile.c_str());
   if( !inFile ) return false;

   // Read the file
   std::string fileLine;
   std::string tmpStr;
   while( std::getline(inFile,fileLine) )
   {
      std::stringstream lineStream(fileLine);
      std::vector<std::string> strVector;
      while( getline(lineStream,tmpStr,' ') ) strVector.push_back(tmpStr);

      // Exit if the line is invalid
      if( strVector.size() < 2 ) break;

      // Add to the map
      fMap[strVector[0]] = strVector[1];
         
   }  // End loop over file
   inFile.close();

   // Create a stringstream for the dimensions
   std::stringstream parseStr(fMap["dims"]);
   tmpStr.clear();
   std::vector<std::string> strVec;
   while(getline(parseStr,tmpStr,',')) strVec.push_back(tmpStr);
   if( strVec.size() < 2) return false;
   m_dimensions.x = std::atof(strVec[0].c_str());
   m_dimensions.y = std::atof(strVec[1].c_str());

   // Create a stringstream for the color
   std::stringstream parseRGB(fMap["color"]);
   tmpStr.clear();
   strVec.clear();
   while(getline(parseRGB,tmpStr,',')) strVec.push_back(tmpStr);
   if( strVec.size() < 3) return false;
   m_color.x = std::atof(strVec[0].c_str());
   m_color.y = std::atof(strVec[1].c_str());
   m_color.z = std::atof(strVec[2].c_str());

   // Set the maximum number of detections
   m_maxDetections = std::atoi(fMap["numDet"].c_str());
 
   return true;
}
