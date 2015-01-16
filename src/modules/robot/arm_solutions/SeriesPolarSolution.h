#ifndef SERIESPOLARSOLUTION_H
#define SERIESPOLARSOLUTION_H
//#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class SeriesPolarSolution : public BaseSolution {
    public:
        SeriesPolarSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );
        
        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);
    private:
        void init();
        float to_degrees(float radians);
        
        float arm1_length;
        float arm2_length;
        float tower_offset_x;
        float tower_offset_y;
        float slow_rate;
};

#endif // SERIESPOLARSOLUTION_H
