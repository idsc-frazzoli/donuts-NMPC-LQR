
/* ************************************
*   Taken from OpenSourceRoboticsFoundation
*
*   link to Source: https://bitbucket.org/osrf/gazebo/src/8bcbc200079f745657357a287998cb667889d577/examples/stand_alone/animated_box/animated_box.cc?at=gazebo_4.0&fileviewer=file-view-default
*
*************************************/


/* ********************************
*
*   Panagiotis Bountouris
*
*   Semester Project: Go-kart modeling and MPC for donuts drifting maneuvers
*
*   Simulate kinematics of Go-kart
*
*********************************** */

/* ********************************
*
*   Comments for Go-Kart team
*
*   modify lines: 50, 82, 85
*
*   line 50: insert file_name.txt
*   line 82: insert length of txt file
*   line 85: insert length of txt file
*
*********************************** */



#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedObj : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      std::vector<std::vector<float> >     data;

          // Replace 'Plop' with your file name.
          std::ifstream          file("radius14maneuver.txt");

          std::string   line;
          // Read one line at a time into the variable line:
          while(std::getline(file, line))
          {
              std::vector<float>   lineData;
              std::stringstream  lineStream(line);

              float value;
              // Read an integer at a time from the line
              while(lineStream >> value)
              {
                  // Add the integers from a line to a 1D array (vector)
                  lineData.push_back(value);
              }

              // When all the integers have been read, add the 1D array
              // into a 2D array (as one line in the 2D array)
              data.push_back(lineData);

          }


      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 46743.0, true));

        gazebo::common::PoseKeyFrame *key;
      for (int i = 0; i < 46743; i++) {
        // set starting location of the box
        key = anim->CreateKeyFrame(i);
        key->Translation(ignition::math::Vector3d(data[i][0], data[i][1], 0.));
        key->Rotation(ignition::math::Quaterniond(0, 0, 1.67+data[i][3]));
      }

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedObj)
}
