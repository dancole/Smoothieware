#include "SeriesPolarSolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
//#include "StreamOutputPool.h"
//#include "Gcode.h"
//#include "SerialMessage.h"
//#include "Conveyor.h"
//#include "Robot.h"
//#include "StepperMotor.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define arm1_length_checksum         CHECKSUM("arm1_length")
#define arm2_length_checksum         CHECKSUM("arm2_length")
#define tower_offset_x_checksum       CHECKSUM("tower_offset_x")
#define tower_offset_y_checksum       CHECKSUM("tower_offset_y")
#define axis_scaling_x_checksum      CHECKSUM("axis_scaling_x")
#define axis_scaling_y_checksum      CHECKSUM("axis_scaling_y")
#define polar_homing_checksum        CHECKSUM("polar_homing")

#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)


SeriesPolarSolution::SeriesPolarSolution(Config* config)
{
    // arm1_length is the length of the upper arm from hinge to hinge
    arm1_length         = config->value(arm1_length_checksum)->by_default(150.0f)->as_number();
    // arm2_length is the length of the forearm main arm from hinge to hinge
    arm2_length         = config->value(arm2_length_checksum)->by_default(150.0f)->as_number();
    // tower_offset_x is the x offset of bed zero position towards the SCARA tower center
    tower_offset_x     = config->value(tower_offset_x_checksum)->by_default(100.0f)->as_number();
    // tower_offset_y is the y offset of bed zero position towards the SCARA tower center
    tower_offset_y     = config->value(tower_offset_y_checksum)->by_default(-65.0f)->as_number();

    init();
}

void SeriesPolarSolution::init() {

}

float SeriesPolarSolution::to_degrees(float radians) {
    return radians*(180.0F/3.14159265359f);
}

void SeriesPolarSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
    // x = arm1_length*cos(S) + arm2_length*cos(S+E)
    // y = arm1_length*sin(S) + arm2_length*sin(S+E)
    // r = S + E + W
    // where S is the upperarm (Shoulder) angle, E is the forearm (Elbow) angle, and W is the tool (Wrist) angle


    float x,
          y,
          shoulder,
          elbow,
          wrist,
          hsq,
          sn,
          sd,
          en,
          ed;

    x = cartesian_mm[X_AXIS] - this->tower_offset_x;
    y = cartesian_mm[Y_AXIS] - this->tower_offset_y;

    hsq = SQ(x) + SQ(y);
    sn = hsq + SQ(this->arm1_length) - SQ(this->arm2_length);
    sd = 2 * this->arm1_length * sqrtf( hsq );
    shoulder =  atan2f(y,x) - acosf(sn/sd);

    en = hsq - SQ(this->arm1_length) - SQ(this->arm2_length);
    ed = 2 * this->arm1_length * this->arm2_length;
    elbow = acosf(en/ed);

    wrist = cartesian_mm[E0_AXIS] - elbow - shoulder;

    actuator_mm[ALPHA_STEPPER] = to_degrees(shoulder);
    actuator_mm[BETA_STEPPER ] = to_degrees(elbow);
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
    // Rotating Tool not supported?
    //actuator_mm[DELTA_STEPPER] = to_degrees(wrist);
}

void SeriesPolarSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] ) {
    // x = arm1_length*cos(S) + arm2_length*cos(S+E)
    // y = arm1_length*sin(S) + arm2_length*sin(S+E)
    // r = S + E + W
    // where S is the upperarm (Shoulder) angle, E is the forearm (Elbow) angle, and W is the tool (Wrist) angle

    cartesian_mm[X_AXIS] = this->arm1_length * cosf(actuator_mm[ALPHA_STEPPER]) + this->arm2_length * cosf(actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER]);
    cartesian_mm[Y_AXIS] = this->arm1_length * sinf(actuator_mm[ALPHA_STEPPER]) + this->arm2_length * sinf(actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER]);
    cartesian_mm[Z_AXIS] = actuator_mm[GAMMA_STEPPER];
    // Rotating Tool not supported?
    //cartesian_mm[E0_AXIS] = actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER] + actuator_mm[DELTA_STEPPER];
}

bool SeriesPolarSolution::set_optional(const arm_options_t& options) {

    arm_options_t::const_iterator i;

    i= options.find('T');          // Theta arm1 length
    if(i != options.end()) {
        arm1_length= i->second;

    }
    i= options.find('P');          // Psi arm2 length
    if(i != options.end()) {
        arm2_length= i->second;
    }
    i= options.find('X');          // Home initial position X
    if(i != options.end()) {
        tower_offset_x= i->second;
    }
    i= options.find('Y');          // Home initial position Y
    if(i != options.end()) {
        tower_offset_y= i->second;
    }
    
    init();
    return true;
}

bool SeriesPolarSolution::get_optional(arm_options_t& options) {
    options['T']= this->arm1_length;
    options['P']= this->arm2_length;
    options['X']= this->tower_offset_x;
    options['Y']= this->tower_offset_y;
    return true;
};
