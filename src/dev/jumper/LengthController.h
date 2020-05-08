#ifndef LENGTH_CONTROLLER_H
#define LENGTH_CONTROLLER_H

/**
 * @file LengthController.h
 * @brief Contains the definition of class LengthController.
 * @author Hany Hamed
 * This was built over one of the source codes of NTRTsim codes
 */


#include "JumperModel.h"
// This library
#include "core/tgObserver.h"
#include "controllers/tgBasicController.h"
#include "core/tgBasicActuator.h"
#include "tgcreator/tgNode.h"
#include "nlohmann/json.hpp"

// The C++ Standard Library
#include <vector>

#include "TCP.h"
#include "JSON_Structure.h"




// Forward declarations
class JumperModel;

class LengthController : public tgObserver<JumperModel>
{
public:
	
	/**
	 * Construct a LengthTensionController.
	 * @param[in] tension, a double specifying the desired tension
	 * throughougt structure. Must be non-negitive
	 */
    LengthController(const char* host, const long long port, int control_type);
    
    /**
     * Nothing to delete, destructor must be virtual
     */
    virtual ~LengthController();
    
    virtual void onSetup(JumperModel& subject);
    
    /**
     * Apply the length controller. Called by notifyStep(dt) of its
     * subject.
     * @param[in] subject - the RPModel that is being controlled. Must
     * have a list of allMuscles populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(JumperModel& subject, double dt);

    virtual void calcTargetLengths(nlohmann::json read_json);

    virtual void controlRestLength(nlohmann::json read_json, double dt, double time);
    virtual void controlRestLength_mod(nlohmann::json read_json, double dt, double time);
    virtual void controlCurrentLength(nlohmann::json read_json, double dt, double time);
    virtual void controlCurrentLength_mod(nlohmann::json read_json, double dt, double time);



    std::vector<tgBasicController*> m_controllers; //instantiate vector of controllers
    std::vector<double> max_lengths; //instantiate vector of random restlengths
    std::vector<tgBasicActuator*> actuators;
    std::vector<tgRod*> rods;
    std::vector<double> target_lengths;

private:
	
    const int port_num;
    const char* host_name;
    double globalTime = 0;
    int toggle = 0;
    int control_type = 1;
    std::vector<int> actuators_states;
    TCP* tcp_com;
};

#endif
