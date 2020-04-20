/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    /**
     * Set the number of particles. Initialize all particles to
     *  - first position (based on estimates of x, y, theta and their uncertainties from GPS)
     *  - and all weights to 1.
     * Add random Gaussian noise to each particle.
     *
     */
    num_particles = 1000;  // Set the number of particles

    for (int i = 0; i < num_particles; ++i)
    {
        /*Initialize all 1000 particles to first position*/
        Particle temp_particle;
        temp_particle.id = i;
        temp_particle.x = x + std[0];
        temp_particle.y = y + std[1];
        temp_particle.theta = theta + std[2];
        temp_particle.weight = 1.0;

        particles.push_back(temp_particle);
        weights.push_back(temp_particle.weight);
    }

    /* Set the initialization flag to TRUE*/
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    /**
     * Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

    std::random_device rd{};
    std::mt19937 gen{ rd() };
    std::normal_distribution<float>d1(0, std_pos[0]);
    std::normal_distribution<float>d2(0, std_pos[1]);
    std::normal_distribution<float>d3(0, std_pos[2]);

    float temp_noise_x = std::round(d1(gen));
    float temp_noise_y = std::round(d2(gen));
    float temp_noise_theta = std::round(d3(gen));

    for (int i = 0; i < particles.size(); ++i)
    {
        if (yaw_rate > 0.0001) /* Avoid division by zero. */
        {
            /* Update distances: https://github.com/arunchavan89/CarND-Kidnapped-Vehicle-Project/blob/master/formulae.pdf */
            particles[i].x = particles[i].x + (velocity / yaw_rate)  * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta)) + temp_noise_x;
            particles[i].y = particles[i].y + (velocity / yaw_rate)  * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t))) + temp_noise_y;

            /* Update orientation */
            particles[i].theta = particles[i].theta + yaw_rate * delta_t + temp_noise_theta;
        }
        else
        {
            /* Update distances */
            particles[i].x = particles[i].x + cos(yaw_rate) * (velocity * delta_t) + temp_noise_x;
            particles[i].y = particles[i].y + sin(yaw_rate) * (velocity * delta_t) + temp_noise_y;

            /* Update orientation */
            particles[i].theta = particles[i].theta + temp_noise_theta;
        }
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
{
    /**
     * Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    const vector<LandmarkObs> &observations,
    const Map &map_landmarks)
{
    /**
     * Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */

    for (int i = 0; i < particles.size(); ++i)
    {
        Particle particle = particles[i];
        double prob = 1.0;

        for (int j = 0; j < observations.size(); j++)
        {
            /* Homogenous Transformation https://github.com/arunchavan89/CarND-Kidnapped-Vehicle-Project/blob/master/formulae.pdf */
            double trans_x = particle.x + (cos(particle.theta) * observations[j].x) - (sin(particle.theta) * observations[j].y);
            double trans_y = particle.y + (sin(particle.theta) * observations[j].x) + (cos(particle.theta) * observations[j].y);

            for (int k = 0; k < map_landmarks.landmark_list.size(); k++)
            {
                /* Calculate distance between particle and landmarks */
                double distance = dist(trans_x, trans_y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
                if (distance <= sensor_range)
                {
                    prob *= calculate_multvariate_normal_distribution(std_landmark[0], std_landmark[1], trans_x, trans_y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
                }
            }

        }
        particles[i].weight = prob;
        weights[i] = prob;
    }
}

void ParticleFilter::resample()
{
    /**
     * Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> discrete_weights(weights.begin(), weights.end());

    std::vector<Particle>resampled_weights;
    for (int i = 0; i < num_particles; i++)
    {
        Particle particle;
        particle = particles[discrete_weights(gen)];
        resampled_weights.push_back(particle);
    }

    particles = resampled_weights;
}

void ParticleFilter::SetAssociations(Particle& particle,
    const vector<int>& associations,
    const vector<double>& sense_x,
    const vector<double>& sense_y)
{
    // particle: the particle to which assign each listed association, 
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

double ParticleFilter::calculate_multvariate_normal_distribution(double std_landmark_x, double std_landmark_y,
    double observation_x, double observation_y, double map_x, double map_y)
{
    /* Multivariate Gaussian probability density: https://github.com/arunchavan89/CarND-Kidnapped-Vehicle-Project/blob/master/formulae.pdf */

    double result = 0.0;
    double term1 = 1 / (2 * M_PI * std_landmark_x *  std_landmark_y);
    double term2 = (observation_x - map_x) * (observation_x - map_x) / 2 * pow(std_landmark_x, 2.0);
    double term3 = (observation_y - map_y) * (observation_y - map_y) / 2 * pow(std_landmark_y, 2.0);
    double exponent_value = exp(-(term2 + term3));
    result = term1 * exponent_value;
    return result;

}