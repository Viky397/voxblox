#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

using namespace std;

inline void IntToRGB(unsigned int rgb, int& r, int& g, int& b)
{
    b = rgb >> 16;
    g = rgb >> 8 & 255;
    r = rgb & 255;
}

inline unsigned int RGBToInt(int r, int g, int b)
{
    unsigned int rgb;
    rgb = r + (unsigned int)(g << 8) + (unsigned int)(b << 16);
    return rgb;
}

class Color {
public:

    bool loadData(const std::string& csv_path) {
        OriLines.clear();
        Labels.clear();
        Dynamicness.clear();
        RGB.clear();
        Color.clear();

        ifstream myFile(csv_path.c_str());
        bool dropFirstline = true;

        string line;

        if(dropFirstline)
            getline(myFile, line);

        int line_num = 0;
        while( getline(myFile, line) )
        {
            if(line.size()==0) continue;
            OriLines.push_back(line);

            vector<string> s;
            boost::split( s, line, boost::is_any_of( "," ), boost::token_compress_on );

            if(s.size() == 6){
                std::string semanticlabel = s[0];
                int r = stoi(s[1]);
                int g = stoi(s[2]);
                int b = stoi(s[3]);
                unsigned int rgb = RGBToInt(r,g,b);
                std::string color = s[4];
                std::string dynamicness = s[5];

                Labels.push_back(semanticlabel);
                Dynamicness.push_back(dynamicness);
                RGB.push_back(rgb);
                Color.push_back(color);
            }

            line_num++;
        }
        myFile.close();

        return true;
    }

    std::string getSemanticLabelFromRGB(int r, int g, int b) {
        unsigned int rgb = RGBToInt(r,g,b);
        auto iter = std::find(RGB.begin(), RGB.end(), rgb);
        if(iter!=RGB.end())
        {
            int id = iter - RGB.begin();
            return Labels[id];
        }

        return "";
    }

    std::string getDynamicNessFromLabel(const std::string& str_label) {
        auto iter = std::find(Labels.begin(), Labels.end(), str_label);
        if(iter!=Labels.end())
        {
            int id = iter - Labels.begin();
            return Dynamicness[id];
        }

        return "";
    }

    int getDynamicValue(const std::string& str_dynamic_ness) {
        if(str_dynamic_ness == "high")
            return 2;
        else if(str_dynamic_ness == "medium")
            return 1;
        else if(str_dynamic_ness == "low")
            return 0;
        else
            throw std::runtime_error("Invalid Dynamic Type: " + str_dynamic_ness);
    }

    int getDynamicTypeFromRGB(int r, int g, int b) {
    	return getDynamicValue(getDynamicNessFromLabel(getSemanticLabelFromRGB(r,g,b)));
    }

    std::vector<std::string> OriLines;

    std::vector<std::string> Labels;
    std::vector<std::string> Dynamicness;
    std::vector<std::string> Color;
    std::vector<unsigned int> RGB;


};

/*
void ParseAllCSV(Colors& parser, bool verbose=false)
{
    std::vector<unsigned int> rgbs = parser.RGB;
    for(int i=0;i<rgbs.size();i++)
    {
        unsigned int rgb = rgbs[i];
        int r,g,b;
        IntToRGB(rgb,r,g,b);
        string label = parser.GetSemanticLabelFromRGB(r,g,b);
        string dynamic_ness = parser.GetDynamicNessFromLabel(label);
        double dynamic_value = parser.GetDynamicValue(dynamic_ness);
        if (verbose) {
			std::cout << "Search RGB : " << r << "," << g << "," << b << std::endl;
			std::cout << "- label : " << label << std::endl;
			std::cout << "- dynamic_ness : " << dynamic_ness << std::endl;
			std::cout << "- dynamic_value : " << dynamic_value << std::endl;
        }
    }
}
*/

/*
int main()
{
    csvParser parser;
    parser.loadData("/home/veronica/kimera_ws/src/Kimera-Semantics/kimera_semantics_ros/cfg/object_class_prior_colour.csv");

    // Test it out!
//    int r,g,b;
//    r= 0;
//    g= 153;
//    b= 153;
//    string label = parser.GetSemanticLabelFromRGB(r,g,b);
//    string dynamic_ness = parser.GetDynamicNessFromLabel(label);
//    double dynamic_value = parser.GetDynamicValue(dynamic_ness);
//    std::cout << "Search RGB : " << r << "," << g << "," << b << std::endl;
//    std::cout << "- label : " << label << std::endl;
//    std::cout << "- dynamic_ness : " << dynamic_ness << std::endl;
//    std::cout << "- dynamic_value : " << dynamic_value << std::endl;

   ParseAllCSV(parser);

    return 0;
}
*/
