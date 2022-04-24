/// \author: Andraz Frelih

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

// hardware interface namespace is the namespace defined by the control_toolbox library
namespace hardware_interface
{

// implementation of a custom hardware interface
class LoadCellArrayHandle
{
public:
  struct Data
  {
    // Note: User-provided constructor required due to a defect in the standard. See https://stackoverflow.com/a/17436088/1932358
    Data() {}

    std::string name;                   //< The name of the sensor
    std::string frame_id;               //< The reference frame to which this sensor is associated
    double* meas          = {nullptr};  //< A pointer to the storage of force measurements: a 4D vector (F1,F2,F3,F4)
  };

  LoadCellArrayHandle(const Data& data = {})
    : name_(data.name),
      frame_id_(data.frame_id),
      meas_(data.meas)
  {}

  LoadCellArrayHandle(
        const std::string& name,        //< The name of the sensor
        const std::string& frame_id,    //< The reference frame to which this sensor is associated
        const double* meas              //< A pointer to the storage of force measurements: a 4D vector (F1,F2,F3,F4)
    )
    : name_(name),
      frame_id_(frame_id),
      meas_(meas)
  {}

  std::string getName()        const {return name_;}
  std::string getFrameId()     const {return frame_id_;}
  const double* getForceMeas() const {return meas_;}


private:
  std::string name_;
  std::string frame_id_;
  const double* meas_;
};

/** \brief Hardware interface to support reading the state of an IMU sensor. */
class LoadCellArrayInterface : public HardwareResourceManager<LoadCellArrayHandle> {};

}
