#pragma once

#include <cstdio>
#include <vector>
#include "ev3cxx.h"
#include <cstdlib>
#include "ev3ys_math.h"

namespace ev3ys
{
    enum colors
    {
        RED,
        BLUE,
        GREEN,
        YELLOW,
        BLACK,
        WHITE
    };

    struct colorspaceRGB{
        double red;
        double green;
        double blue;
        double white;
    };

    struct colorspaceHSV{
        double hue;
        double saturation;
        double value;
    };

    struct color_hue
    {
        colors color;
        double hue;
        double zoneSize;
    };

    struct colorCalibration
    {
        std::vector <color_hue> hues;
        double minColorSaturation;
        double greyscaleIntersection;
    };

    class colorSensor : protected ev3cxx::ColorSensor
    {
    private:
        char port[8];
        char* dir;
        double offsetRef, scaleRef;
        double offsetRGB[4] = {0, 0, 0, 0}, scaleRGB[4] = {0, 0, 0, 0};
        bool normalisedRef, normalisedRGB;
        colorCalibration colorData;
        bool lineDetected;
        int cutoffValue;
        colorspaceRGB rgb;
        colorspaceHSV hsv;
        colors col;
        ev3cxx::Bluetooth bt;
        ev3ys::timer t;

        int lastSample;
        int filterValue;
        bool resetFilter;
        bool filter;

    public:
        colorSensor(ev3cxx::SensorPort port, bool normalised, char *calDir, int colorsAmount = 5) : ev3cxx::ColorSensor(port) //ATTENTION!!! calDir must exist inside SDcard root directory
        {
            sprintf(this->port, "S%d", ((int)port)+1);
            dir = calDir;
            offsetRef = 0;
            scaleRef = 1;
            lineDetected = false;

            color_hue cols[5];
            cols[0] = {RED, 5, 10};
            cols[1] = {RED, 330, 60};
            cols[2] = {BLUE, 210, 80};
            cols[3] = {GREEN, 120, 80};
            cols[4] = {YELLOW, 40, 60};
            colorData.minColorSaturation = 50;
            colorData.greyscaleIntersection = 18;
            for(int i = 0; i < 5; i++)
            {
                colorData.hues.push_back(cols[i]);
            }

            setNormalisation(normalised);
            setCutoffValue(10);
            
            rgb = {0, 0, 0, 0};
            hsv = {0, 0, 0};
            col = colors::BLACK;

            lastSample = 0;
            resetFiltering();
            setFiltering(true, 4);
        }

        void setNormalisation(bool normalised, int colorsAmount = 5)
        {
            if(normalisedRGB == normalised) return;
            normalisedRef = normalised;
            normalisedRGB = normalised;
            if (normalised)
            {
                loadRefCalParams();
                loadRgbCalParams();
                loadColorCalParams(colorsAmount);
            }
        }

        void setFiltering(bool filtering, int filterValue = 0)
        {
            resetFiltering();
            this->filterValue = filterValue;
            filter = filtering;
        }

        void resetFiltering()
        {
            resetFilter = true;
        }

        void setCutoffValue(int value)
        {
            cutoffValue = value;
        }

        int getCutoffValue()
        {
            return cutoffValue;
        }

        bool getLineDetected()
        {
            return lineDetected;
        }

        int getReflected()
        {
            int sample = reflected();
            if (normalisedRef)
            {
                sample = (int)((offsetRef + sample) * scaleRef);
                sample = clamp(sample, 0, 100);
            }

            if(filter)
            {
                if(abs(sample - lastSample) > filterValue)
                {
                    lineDetected = sample <= cutoffValue;
                    if(!resetFilter)
                        sample = 60;
                }
                else
                {
                    lineDetected = false;
                }
            }
            else
            {
                lineDetected = sample <= cutoffValue;
            }
            
            resetFilter = false;
            lastSample = sample;

            return sample;
        }

        colorspaceRGB& getRGB()
        {	
            rgb_raw_t values = reflectedRawRgb();
            //t.secDelay(0.001);
            tslp_tsk(10);
            
            if (normalisedRGB)
            {
                rgb.white = values.r + values.g + values.b;
                rgb.white = (offsetRGB[3] + rgb.white) * scaleRGB[3];
                rgb.red = (offsetRGB[0] + values.r) * scaleRGB[0];
                rgb.green = (offsetRGB[1] + values.g) * scaleRGB[1];
                rgb.blue = (offsetRGB[2] + values.b) * scaleRGB[2];
                rgb.white = clamp((int)rgb.white, 1, 100);
                rgb.red = clamp((int)rgb.red, 1, 255);
                rgb.green = clamp((int)rgb.green, 1, 255);
                rgb.blue = clamp((int)rgb.blue, 1, 255);
            }
            else
            {
                rgb.white = values.r + values.g + values.b;
                rgb.red = clamp(values.r, 1, 255);
                rgb.green = clamp(values.g, 1, 255);
                rgb.blue = clamp(values.b, 1, 255);
            }  

            if(filter)
            {
                if(abs(rgb.white - lastSample) > filterValue)
                {
                    lineDetected = rgb.white <= cutoffValue;
                    if(!resetFilter)
                        rgb.white = 60;
                }
                else
                {
                    lineDetected = false;
                }
            }
            else
            {
                lineDetected = rgb.white <= cutoffValue;
            }
            
            resetFilter = false;
            lastSample = rgb.white;
            
            return rgb;
        }

        colorspaceHSV& getHSV()
        {
            rgb = getRGB();
            rgb.red /= 255.0;
            rgb.green /= 255.0;
            rgb.blue /= 255.0;

            double minV, maxV, delta;
            minV = std::min(rgb.red, rgb.green);
            minV = std::min(minV, rgb.blue);
            maxV = std::max(rgb.red, rgb.green);
            maxV = std::max(maxV, rgb.blue);
            hsv.value = maxV * 100;
            delta = maxV - minV;
            if (maxV != 0)
                hsv.saturation = (delta / maxV) * 100;
            else
            {
                hsv.saturation = 0;
                hsv.hue = -1;
                return hsv;
            }
            if (rgb.red == maxV)
                hsv.hue = (rgb.green - rgb.blue) / delta;
            else if (rgb.green == maxV)
                hsv.hue = 2.0 + (rgb.blue - rgb.red) / delta;
            else
                hsv.hue = 4.0 + (rgb.red - rgb.green) / delta;
            hsv.hue *= 60;
            if (hsv.hue < 0)
                hsv.hue += 360;
            return hsv;
        }

        colors& getColor()
        {
            // hsv = getHSV();

            // if(hsv.saturation < colorData.minColorSaturation)
            // {
            //     if(hsv.value > colorData.greyscaleIntersection)
            //         col = colors::WHITE;
            //     else
            //         col = colors::BLACK;
            // }
            // else
            // {
            //     for(auto colorProt: colorData.hues)
            //     {
            //         if(checkRange(hsv.hue, colorProt.hue, colorProt.zoneSize))
            //             col = colorProt.color;
            //     }
            // }

            switch(color())
            {
                case COLOR_RED:
                    col = RED;
                    break;
                case COLOR_BLUE:
                    col = BLUE;
                    break;
                case COLOR_GREEN:
                    col = GREEN;
                    break;
                case COLOR_YELLOW:
                    col = YELLOW;
                    break;
                case COLOR_BLACK:
                    col = BLACK;
                    break;
                case COLOR_WHITE:
                case COLOR_BROWN:
                case COLOR_NONE:
                    col = WHITE;
                    break;
            }
            
            return col;
        }

        void setRefCalParams(double min, double max)
        {
            tslp_tsk(1);
            offsetRef = -min;
            scaleRef = 100.0 / (max - min);
            char filename[32];
            sprintf(filename, "%s/%srefCal.txt", dir, port);
            FILE *calData = fopen(filename, "w+");
            fprintf(calData, "%lf\n%lf\n", offsetRef, scaleRef);
            fclose(calData);
        }

        void loadRefCalParams()
        {
            tslp_tsk(1);
            char filename[32];
            sprintf(filename, "%s/%srefCal.txt", dir, port);
            FILE *calData = fopen(filename, "r+");
            if(calData != nullptr)
            {
                fscanf(calData, "%lf", &offsetRef);
                fscanf(calData, "%lf", &scaleRef);
            }
            else
                normalisedRef = false;
            
            fclose(calData);
        }

        void setRgbCalParams(const double *min, const double *max)
        {
            tslp_tsk(1);
            char filename[32];
            sprintf(filename, "%s/%srgbCal.txt", dir, port);
            FILE *calData = fopen(filename, "w+");
            for (int i = 0; i < 3; i++)
            {
                offsetRGB[i] = -min[i];
                scaleRGB[i] = 255.0 / (max[i] - min[i]);
                fprintf(calData, "%lf\n%lf\n", offsetRGB[i], scaleRGB[i]);
            }
            offsetRGB[3] = -min[3];
            scaleRGB[3] = 100.0 / (max[3] - min[3]);
            fprintf(calData, "%lf\n%lf\n", offsetRGB[3], scaleRGB[3]);
            fclose(calData);
        }

        void loadRgbCalParams()
        {
            tslp_tsk(1);
            char filename[32];
            sprintf(filename, "%s/%srgbCal.txt", dir, port);
            FILE *calData = fopen(filename, "r+");
            if(calData != nullptr)
            {
                for (int i = 0; i < 4; i++)
                {
                    fscanf(calData, "%lf", &offsetRGB[i]);
                    fscanf(calData, "%lf", &scaleRGB[i]);
                }
            }
            else
                normalisedRGB = false;
            
            fclose(calData);
        }

        void setColorCalParams(color_hue *hues, int colorsAmount, double minColorSaturation, double greyscaleIntersection)
        {
            tslp_tsk(1);
            char filename[32];
            sprintf(filename, "%s/%scolorCal.txt", dir, port);
            FILE *calData = fopen(filename, "w+");
            
            colorData.hues.clear();
            fprintf(calData, "%lf\n%lf\n", minColorSaturation, greyscaleIntersection);
            colorData.minColorSaturation = minColorSaturation;
            colorData.greyscaleIntersection = greyscaleIntersection;

            for(int i = 0; i < colorsAmount; i++)
            {
                colorData.hues.push_back(hues[i]);
                fprintf(calData, "%d\n%lf\n%lf\n", static_cast<int>(hues[i].color), hues[i].hue, hues[i].zoneSize);
            }

            fclose(calData);
        }

        void loadColorCalParams(int colorsAmount)
        {
            tslp_tsk(1);
            char filename[32];
            sprintf(filename, "%s/%scolorCal.txt", dir, port);
            FILE *calData = fopen(filename, "r+");

            colorData.hues.clear();
            if(calData != nullptr)
            {
                fscanf(calData, "%lf", &colorData.minColorSaturation);
                fscanf(calData, "%lf", &colorData.greyscaleIntersection);
                for (int i = 0; i< colorsAmount; i++)
                {
                    color_hue data;
                    fscanf(calData, "%d", &data.color);
                    fscanf(calData, "%lf", &data.hue);
                    fscanf(calData, "%lf", &data.zoneSize);
                    colorData.hues.push_back(data);
                }
            }
            fclose(calData);
        }
    };
}
