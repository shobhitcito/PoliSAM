/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include "abb_libegm/egm_controller_interface.h"

/**
 * \brief Generates mockup velocity references for simulating tracking an object ontop of a conveyor belt.
 *
 * \param delta_seconds specifying the seconds since the start of the EGM communication session.
 * \param feedback containing the latest received EGM feedback.
 * \param cartesian_velocity for containing the generated EGM velocity references.
 */
void mockupGenerateVelocity(const unsigned int delta_seconds,
                            const abb::egm::wrapper::Feedback& feedback,
                            abb::egm::wrapper::CartesianVelocity* cartesian_velocity)
{
  if (delta_seconds >= 0 && cartesian_velocity)
  {
    // Generate different velocity references according to the predefined time periods.
    // Note: The references are related to the workobject and tooldata defined in the RAPID program.
    if (delta_seconds < 3)
    {
      cartesian_velocity->mutable_linear()->set_x(0.8*(100.0 - feedback.robot().cartesian().pose().position().x()));
      cartesian_velocity->mutable_linear()->set_y(150.0);
      cartesian_velocity->mutable_linear()->set_z(0.0);

      cartesian_velocity->mutable_angular()->set_z(2.0*(90.0 - feedback.robot().cartesian().pose().euler().z()));
    }
    else if (delta_seconds < 4)
    {
      cartesian_velocity->mutable_linear()->set_x(0.0);
      cartesian_velocity->mutable_linear()->set_y(150.0);
      cartesian_velocity->mutable_linear()->set_z(4.0*(-20.0 - feedback.robot().cartesian().pose().position().z()));

      cartesian_velocity->mutable_angular()->set_z(0.0);
    }
    else if (delta_seconds < 5)
    {
      cartesian_velocity->mutable_linear()->set_x(0.0);
      cartesian_velocity->mutable_linear()->set_y(150.0);
      cartesian_velocity->mutable_linear()->set_z(4.0*(-120.0 - feedback.robot().cartesian().pose().position().z()));

      cartesian_velocity->mutable_angular()->set_z(0.0);
    }
    else if (delta_seconds < 7)
    {
      cartesian_velocity->mutable_linear()->set_x(0.8*(0.0 - feedback.robot().cartesian().pose().position().x()));
      cartesian_velocity->mutable_linear()->set_y(150.0);
      cartesian_velocity->mutable_linear()->set_z(0.0);

      cartesian_velocity->mutable_angular()->set_z(2.0*(0.0 - feedback.robot().cartesian().pose().euler().z()));
    }
    else
    {
      cartesian_velocity->mutable_linear()->set_x(0.0);
      cartesian_velocity->mutable_linear()->set_y(0.0);
      cartesian_velocity->mutable_linear()->set_z(0.0);

      cartesian_velocity->mutable_angular()->set_z(0.0);
    }
  }
}

/**
 * \brief Sets up and executes an EGM controller.
 */
void mockupEGMController()
{
  // Boost components for managing the asynchronous UDP socket.
  boost::asio::io_service io_service;
  boost::thread_group worker_threads;

  // Configuration for allowing EGM velocity refernces.
  abb::egm::BaseConfiguration configuration;
  configuration.use_velocity_outputs = true;

  // Create an EGN controller interface (includes an EGM server).
  abb::egm::EGMControllerInterface egm_interface(io_service, 6511, configuration);
  worker_threads.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  // Data containers.
  bool done = false;
  bool first_message = true;
  unsigned int delta_seconds = 0;
  abb::egm::wrapper::Input initial_input;
  abb::egm::wrapper::Input input;
  abb::egm::wrapper::Output output;

  // Run the control loop.
  while (!done)
  {
    // Wait for an EGM message.
    if (egm_interface.waitForMessage(500))
    {
      // Read the message.
      egm_interface.read(&input);

      // Store the initial input message.
      if (first_message)
      {
        initial_input = input;
        first_message = false;
      }

      delta_seconds = input.feedback().time().sec() - initial_input.feedback().time().sec();

      // Generate mockup velocity references.
      mockupGenerateVelocity(delta_seconds,
                             input.feedback(),
                             output.mutable_robot()->mutable_cartesian()->mutable_velocity());

      // Write the reference message to the EGM client (inside the robot controller).
      egm_interface.write(output);

      done = delta_seconds > 10;
    }
  }

  worker_threads.join_all();
}

/**
 * \brief Main function.
 */
int main()
{
  mockupEGMController();
  return 0;
}