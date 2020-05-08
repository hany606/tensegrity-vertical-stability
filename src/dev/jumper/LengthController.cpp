/**
 * @file LengthController.h
 * @brief Implementation of class LengthController
 * @author Hany Hamed
 * This was built over one of the source codes of NTRTsim codes
 */

// This module
#include "LengthController.h"
#include <core/tgSpringCableActuator.h>


// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <algorithm>

// #define HOST_NAME "localhost"
// #define PORT_NUM 10042
#define MAX_BUFF_SIZE 5000
#define EPS 0.00001  
#define SMALL_EPS(eps) eps/1000.0
#define BIG_EPS(eps) eps*10.0
#define MAX_LENGTH_PERCENT 1

using namespace std;
using json = nlohmann::json;

json read_json;
bool all_reached_target = true;
// double last_all_reached_time = 0;   // This is used to indicate if there is a stuck or not in the length of the cable
vector <double> last_error;  //actuators.size()
vector <btVector3> last_positions;


// It has been extracted by some nodes that connect betwee nthe controllers of the cabels
//    and some nodes has been swaped inplace in order to get all the nodes in the first index of the 
//    anchor list to uniform and ease the access to the data 
int end_points_map[]={0,1,2,3,0,4};

// control_type: 0 for rest_length control and 1 for current_length control
LengthController::LengthController(const char* host, const long long port, int control_type): host_name(host), port_num(port), control_type(control_type){
  if(control_type == 0)
    printf("The controller is on the rest_length mode\n");
  else if(control_type == 1)
    printf("The controller is on the current_length mode\n");
  else if(control_type == 2)
    printf("The controller is on the rest_length_mod mode\n");  
  else if(control_type == 3)
    printf("The controller is on the current_length_mod mode\n");
  else
    throw std::invalid_argument( "control_type should be 0 for rest_length or 1 for current_length\n You choosed sometthing different in App.....cpp please change it\n" );

}

LengthController::~LengthController()
{
}	

void LengthController::onSetup(JumperModel& subject)
{
  // freopen("records_testing/record.txt","w",stdout); //For debugging
  std::cout<<"\nStarting communication through TCP: "<<host_name<<port_num<<"\n";//DEBUG
  LengthController::tcp_com = new TCP(host_name, port_num);
  LengthController::tcp_com->setup();
  std::cout<<"Finished Setup the communication\n";//DEBUG

  // printf("LengthController is working now\n");


  JSON_Structure::setup();
  m_controllers.clear(); //clear vector of controllers
  max_lengths.clear(); //vector of randomized restlengths
    
  //get all of the tensegrity structure's cables
  actuators = subject.getAllActuators();
  rods = subject.getAllRods();

  printf("Number of actuators: %d , Number of Rods: %d\n", (int) actuators.size(), (int) rods.size());//DEBUG
  // std::cout<<rods[1]->getTags()[0][1]<<"\n";

  //Attach a tgBasicController to each actuator
  for (size_t i = 0; i < actuators.size(); ++i)
  {
    tgBasicActuator * const pActuator = actuators[i];
    assert(pActuator != NULL);  //precondition
    //instantiate controllers for each cable
    tgBasicController* m_lenController = new tgBasicController(pActuator);
    //add controller to vector
    m_controllers.push_back(m_lenController);
    // getStartLength
    double start_length = actuators[i]->getRestLength();
    printf("Actutor of string #%d -> start Lenght: %lf\n", (int) i, start_length);//DEBUG
    max_lengths.push_back(MAX_LENGTH_PERCENT*start_length);
    actuators_states.push_back(0);
    target_lengths.push_back(0);
    last_error.push_back(0);
  }
  btVector3 end_point = actuators[0]->getAnchors_mod()[1]->getWorldPosition();
  last_positions.push_back(end_point);
  for(int i = 1; i < 6; i++){
    btVector3 end_point = actuators[end_points_map[i]]->getAnchors_mod()[0]->getWorldPosition();
    last_positions.push_back(end_point);
  }

  
}

//This function is being activated each step
void LengthController::onStep(JumperModel& subject, double dt)
{

  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    globalTime += dt;
    if(globalTime > 0){ //delay start of cable actuation
      if(toggle==0){    //print once when motors start moving
        std::cout<<"Working...\n";
        //DEBUG
        
        cout << endl << "Activating Cable Motors -------------------------------------" << endl;
        std::cout<<"End Point"<<0<<"\nPoint:"<<actuators[0]->getAnchors_mod()[1]->getWorldPosition()<<"\n----------------------\n";
        for(int i = 1; i < 6; i++){
            btVector3 end_point = actuators[end_points_map[i]]->getAnchors_mod()[0]->getWorldPosition();
            std::cout<<"End Point"<<i<<"\nPoint:"<<end_point<<"\n----------------------\n";
        }

        for(int i = 0; i < 8; i++){
          std::cout<< "String #"<<i<<": "<<((int) (actuators[i]->getRestLength()*10000)/10000.0)<<"\n";
          std::cout<< "Max length: "<<max_lengths[i]<<"\n";

        }
        

        // std::cout<<"CMS: "<<rods[0]->centerOfMass()<<"\tPoint1:"<<actuators[3]->getAnchors_mod()[0]->getWorldPosition()<<"or:"<<actuators[3]->getAnchors_mod()[1]->getWorldPosition()<<"\tPoint2:"<<actuators[0]->getAnchors_mod()[0]->getWorldPosition()<<"or:"<<actuators[0]->getAnchors_mod()[1]->getWorldPosition()<<"\n";
        // std::cout<<rods[1]->length()<<"\n";
        toggle = 1;   //is used like a state flag ---- set it to 2 to disable the movement
      }
      // Debugging mode
      if(toggle == 2){
        //actuators[0] between point 1,6
        //actuators[3] between point 0,4
        //rod[0] between point 0,1
        // std::cout<<"CMS: "<<rods[0]->centerOfMass()<<"\nPoint1:"<<actuators[3]->getAnchors_mod()[0]->getWorldPosition()<<"\nPoint2:"<<actuators[0]->getAnchors_mod()[0]->getWorldPosition()<<"\n";
        // std::cout<<rods[1]->length()<<"\n";
        // std::cout<<rods[1]->getPRigidBody()<<"\n";
      }
      /**
       * Observations:
       *    1 - Cables' lengths
       *    2 - Center of Mass for rods
       *    3 - Time
       * */
      
      if(toggle == 1){

        // Part 1: Read the upcoming orders from the python module
        if(all_reached_target == true){
          char buffer[MAX_BUFF_SIZE];
          bzero(&buffer,MAX_BUFF_SIZE);
          // std::cout<<"Waiting for the TCP to read\n";        //DEBUG
          // std::cout<<".\n";
          LengthController::tcp_com->read_TCP(buffer,MAX_BUFF_SIZE);
          // std::cout<<"Buffer ::";//DEBUG
          // std::cout<<buffer<<"\n";//DEBUG
          read_json = JSON_Structure::stringToJson(buffer);
        }

        if(LengthController::control_type == 0)
          LengthController::controlRestLength(read_json, dt, globalTime);
        else if(LengthController::control_type == 1)
          LengthController::controlCurrentLength(read_json, dt, globalTime);
        else if(LengthController::control_type == 2)
          LengthController::controlRestLength_mod(read_json, dt, globalTime);
        else if(LengthController::control_type == 3)
          LengthController::controlCurrentLength_mod(read_json, dt, globalTime);



        if(all_reached_target == true){

          //If the getRelativePosition is not working properly
          // double total_mass = 0;
          // btVector3 first_moment_system;
          // //Calculate the center of the mass of the whole strucutre
          // for(int i = 0; i < rods.size(); i++){
          //   std::cout<<"CoM: "<<i<<":"<<rods[i]->centerOfMass()<<"\n";
          //   first_moment_system = rods[i]->mass()*rods[i]->centerOfMass();
          //   total_mass += rods[i]->mass();
          // }
          // btVector3 CoM = first_moment_system/total_mass;
          btVector3 CoM = rods[0]->centerOfMass();
          // std::cout<<"CoM: "<<CoM<<"\n";
          // If the relative position is not working: actuators[0]->getAnchors_mod()[1]-CoM;

          // printf("\n--------------------------------------------------------\n");//DEBUG
          // (1) Get the end-points
          // getWorldPosition()
          
          btVector3 end_point = actuators[0]->getAnchors_mod()[1]->getWorldPosition();
          
          btVector3 end_point_relative = end_point - CoM;
          JSON_Structure::setEndPoints(0,end_point_relative);
          btVector3 end_point_velocity = (end_point - last_positions[0])/dt;
          JSON_Structure::setEndPointVelocity(0,end_point_velocity);
          last_positions[0] = end_point;

          for(int i = 1; i < 6; i++){
            btVector3 end_point = actuators[end_points_map[i]]->getAnchors_mod()[0]->getWorldPosition();
            if(i == 4 || i == 5)
              JSON_Structure::setLegEndPoints(i-4, end_point);
            btVector3 end_point_relative = end_point - CoM;
            JSON_Structure::setEndPoints(i,end_point_relative);
            // Calculate the velocities of the end points (nodes of the rods)
            btVector3 end_point_velocity = (end_point - last_positions[i])/dt;
            JSON_Structure::setEndPointVelocity(i,end_point_velocity);
            last_positions[i] = end_point;
            // std::cout<<"End Point"<<i<<"\nPoint:"<<end_point<<"\n----------------------\n";
          }


          // (2) Get the length of targeting cables
          for(int i = 0; i < actuators.size(); i++){
            JSON_Structure::setRestCableLength(i, (int) (actuators[i]->getRestLength()*10000) /10000.0);
            JSON_Structure::setCurrentCableLength(i, (int) (actuators[i]->getCurrentLength()*10000) /10000.0);
          }

          // (3) Get the reached flag
          JSON_Structure::setFlags(0, (int) all_reached_target);

          // (4) Set the time stamp
          JSON_Structure::setTime(((int)(globalTime*1000)) /1000.0);

          std::string json_string = JSON_Structure::jsonToString();
          // std::cout<<"String to be sent"<<json_string<<std::endl;
          LengthController::tcp_com->write_TCP((void*) json_string.c_str());
        }
      }

      
    }
  }

}

void LengthController::calcTargetLengths(json read_json){
  for(int i = 0; i < actuators.size(); i++){
      // Discrete action space and for cotinous delta lengths
      target_lengths[i] = actuators[i]->getRestLength() + (double)read_json["Controllers_val"][i];
      if (target_lengths[i] < 0.0) {
        target_lengths[i] = 0.0;
      }
      // Only this clamping while controlling the rest_lengths
      if ((LengthController::control_type == 0 || LengthController::control_type == 2) && target_lengths[i] > max_lengths[i]){
        target_lengths[i] = max_lengths[i];
        printf("Reached the limit\n");
      }

      // Continuous action space for lengths
      // target_lengths[i] = (double)read_json["Controllers_val"][i];

    }
}

void LengthController::controlRestLength(json read_json, double dt, double time){

  //set new targets
  if(all_reached_target == true){
    all_reached_target = false;
    LengthController::calcTargetLengths(read_json);
  }

  int counter = 0;
  int reached_counter = 0;
  for(int i = 0; i < actuators.size(); i++){
    if(((double) read_json["Controllers_val"][i]) == 0)
      continue;
    
    counter++;
    double error_sign = actuators[i]->getRestLength() - target_lengths[i];
    double error = fabs(error_sign);
    // if(error == last_error[i]){
    double stuck_err = last_error[i] - error;
    // that the error is equal to the last_error and the last_error was greater than the current error and the error was decreasing and the target length is smaller than the current which means that it is going to decrease more
    if( (actuators[i]->getRestLength() == 0.1  && target_lengths[i] <= actuators[i]->getRestLength()) ||(fabs(stuck_err) < SMALL_EPS(EPS) && stuck_err > 0 && error_sign > 0 )){ //changed
      // while (1);
      printf("!!!!Stuck: %d\n",i);
      // all_reached_target = true;  //TODO: This is wrong, it should just flag the controller reach flag not all
      reached_counter++;
      printf("Controller#%d\tError: %lf\n", i, error);
      std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;
      continue;
    }
    printf("Controller#%d\tError: %lf\n", i, error);
    std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;

    // m_controllers[i]->control(dt,((double) read_json["Controllers_val"][i]));
    m_controllers[i]->control(dt, target_lengths[i]);
    actuators[i]->moveMotors(dt);
    // printf("%d\n", actuators.size());
    // printf("#%d -> %lf\n, -> %lf", i, (double) read_json["Controllers_val"][i], 5);
    // printf("ERR:%lf\n",abs(actuators[i]->getCurrentLength()- (double)read_json["Controllers_val"][i]));
    if(error <= EPS){
      // all_reached_target = true;
      reached_counter++;
      read_json["Controllers_val"][i] = 0;
      printf("Reached%d\n", i);
    }
    last_error[i] = error;
  }

  if(reached_counter == counter)
    all_reached_target = true;
  if(counter == 0)
    all_reached_target = true;
  if(all_reached_target == true){
      for(int i = 0; i < actuators.size(); i++)
        last_error[i] = 0;
  }
}


void LengthController::controlRestLength_mod(json read_json, double dt, double time){

  //set new targets
  if(all_reached_target == true){
    // all_reached_target = false;
    LengthController::calcTargetLengths(read_json);
  }

  for(int i = 0; i < actuators.size(); i++){
    if(((double) read_json["Controllers_val"][i]) == 0)
      continue;

    std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;

    // m_controllers[i]->control(dt,((double) read_json["Controllers_val"][i]));
    m_controllers[i]->control(dt, target_lengths[i]);
    actuators[i]->moveMotors(dt);
    // printf("%d\n", actuators.size());
    // printf("#%d -> %lf\n, -> %lf", i, (double) read_json["Controllers_val"][i], 5);
    // printf("ERR:%lf\n",abs(actuators[i]->getCurrentLength()- (double)read_json["Controllers_val"][i]));
  }

}



void LengthController::controlCurrentLength(json read_json, double dt, double time){
  //set new targets
  if(all_reached_target == true){
    all_reached_target = false;
    LengthController::calcTargetLengths(read_json);
  }

  int counter = 0;
  int reached_counter = 0;
  for(int i = 0; i < actuators.size(); i++){
    if(((double) read_json["Controllers_val"][i]) == 0)
      continue;
    
    counter++;
    double error_sign = actuators[i]->getCurrentLength() - target_lengths[i];
    double error = fabs(error_sign);
    double stuck_err = last_error[i] - error;

    // Stuck when on of them satisfied
    //- Converge; The difference between the iteration in the error is less than epsilon (converge to specific length after a specific move)
    //- Error is increasing; The current error is greater than the previous error + eps
    if((fabs(last_error[i]) + BIG_EPS(EPS)*10 <= fabs(error) && last_error[i] != 0.0) || fabs(stuck_err) <= BIG_EPS(EPS)){
      printf("!!!!Stuck: %d\n",i);
      // all_reached_target = true;  //TODO: This is wrong, it should just flag the controller reach flag not all
      reached_counter++;
      printf("Controller#%d\tError: %lf\tlast Error: %lf\n", i, error_sign, last_error[i]);
      std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;
      continue;
    }
    printf("Controller#%d\tError: %lf\tlast Error: %lf\n", i, error_sign, last_error[i]);
    std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;

    // m_controllers[i]->control(dt,((double) read_json["Controllers_val"][i]));
    m_controllers[i]->control(dt, target_lengths[i]);
    // printf("%d\n", actuators.size());
    // printf("#%d -> %lf\n, -> %lf", i, (double) read_json["Controllers_val"][i], 5);
    // printf("ERR:%lf\n",abs(actuators[i]->getCurrentLength()- (double)read_json["Controllers_val"][i]));
    if(error <= EPS){
      // all_reached_target = true;
      reached_counter++;
      read_json["Controllers_val"][i] = 0;
      printf("Reached%d\n", i);
    }
    last_error[i] = error;
  }
  if(reached_counter == counter)
    all_reached_target = true;

  if(counter == 0)
    all_reached_target = true;

  if(all_reached_target == true){
      for(int i = 0; i < actuators.size(); i++)
        last_error[i] = 0;
  }
}




void LengthController::controlCurrentLength_mod(json read_json, double dt, double time){
  //set new targets
  if(all_reached_target == true){
    // all_reached_target = false;
    LengthController::calcTargetLengths(read_json);
  }

  for(int i = 0; i < actuators.size(); i++){
    if(((double) read_json["Controllers_val"][i]) == 0)
      continue;
    
    std::cout<<"Current Length: "<<actuators[i]->getCurrentLength()<<"\tRest Length: "<<actuators[i]->getRestLength()<<"\tTarget: "<<target_lengths[i]<<std::endl;
    m_controllers[i]->control(dt, target_lengths[i]);
    // printf("%d\n", actuators.size());
    // printf("#%d -> %lf\n, -> %lf", i, (double) read_json["Controllers_val"][i], 5);
    // printf("ERR:%lf\n",abs(actuators[i]->getCurrentLength()- (double)read_json["Controllers_val"][i]));
    printf("Reached%d\n", i);  
  }
  // all_reached_target = true;

}

